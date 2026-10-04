import { selectBest, displayResult, layoutToJSON } from '../io/results.js';
import { importLayout, parseLayoutJSON } from '../io/layout-import.js';
import { parseSimulatorSnapshot, parseWorkspaceRanges, workspaceRangesToJSON } from '../simulator/snapshot.js';
import { download } from './download.js';
import { parseRequirements } from '../model/requirements.js';
import { loadDefaultRequirements as loadSample } from '../io/sample-requirements.js';
import { Optimizer as DefaultOptimizer } from '../optimization/optimizer.js';
import { WorkerOptimizer } from './worker-optimizer.js';
import { progressSnapshot } from './worker-protocol.js';
import { createRunDashboard } from './run-dashboard.js';
import { createControls } from './controls.js';
import { installTooltips } from './tooltips.js';
import { createResultsView } from './results-view.js';
import { buildConstructionSkeleton, canExportCad, skeletonToCSV, skeletonToFusionScript } from '../io/cad.js';
import { createServoRatingControls } from './servo-ratings-controls.js';
import { createSimulatorController } from '../simulator/controller.js';
import { createSimulatorView } from '../simulator/view.js';
import { createGeometryControls } from '../simulator/geometry-controls.js';
import { mountSimulatorDiagnostics } from '../simulator/diagnostics.js';
import { loadModelFromSettings } from '../simulator/loads.js';
import { LOCAL_WORKSPACE_KEY, captureLocalWorkspace, parseLocalWorkspace,
    applyLocalWorkspace } from './local-workspace.js';
import { DEFAULT_BALL_JOINT_LIMIT_DEG, DEFAULT_LINK_CLEARANCE_MM } from '../contracts.js';

function simulatorOptions(settings = {}, layout = {}) {
    return {
        ballJointLimitDeg: settings.ballJointLimitDeg ?? DEFAULT_BALL_JOINT_LIMIT_DEG,
        lowerBallJointLimitDeg: settings.lowerBallJointLimitDeg ?? settings.ballJointLimitDeg ?? DEFAULT_BALL_JOINT_LIMIT_DEG,
        upperBallJointLimitDeg: settings.upperBallJointLimitDeg ?? settings.ballJointLimitDeg ?? DEFAULT_BALL_JOINT_LIMIT_DEG,
        conditionLimit: settings.conditionLimit ?? null,
        linkClearanceMm: settings.linkClearanceMm ?? DEFAULT_LINK_CLEARANCE_MM,
        servoRangeRad: layout.servoRangeRad,
        rodLengthTolerance: settings.rodLengthTolerance ?? 0.5,
    };
}

export function createApp({ document, window, Optimizer = DefaultOptimizer, workerFactory,
    loadDefaultRequirements = loadSample, downloadFile = download,
    ChartClass = globalThis.Chart,
    now = () => performance.now() }) {
    const requirementsInput = document.getElementById('requirementsInput');
    const referenceLayoutInput = document.getElementById('referenceLayoutInput');
    const statusEl = document.getElementById('optStatus');
    const resultOutput = document.getElementById('resultOutput');
    const copyResultButton = document.getElementById('copyResultOutput');
    const copyResultStatus = document.getElementById('copyResultStatus');
    function setResultOutput(value) {
        resultOutput.value = value;
        copyResultButton.disabled = !value;
        copyResultStatus.textContent = '';
    }
    setResultOutput('');
    copyResultButton.addEventListener('click', async () => {
        const text = resultOutput.value;
        if (!text) return;
        let copied = false;
        try {
            if (window.navigator?.clipboard?.writeText) {
                await window.navigator.clipboard.writeText(text);
                copied = true;
            }
        } catch { /* Try the selection fallback for browsers that block Clipboard API writes. */ }
        if (!copied) {
            try {
                resultOutput.focus();
                resultOutput.select();
                copied = document.execCommand('copy');
                copyResultButton.focus();
            } catch { /* The user can still select and copy the JSON manually. */ }
        }
        if (resultOutput.value === text) {
            copyResultStatus.textContent = copied
                ? 'Copied to clipboard.'
                : 'Copy blocked. Select the JSON and copy it manually.';
        }
    });
    const ballJointClampCheckbox = document.getElementById('ballJointClamp');
    const ballJointLimitInput = document.getElementById('ballJointLimit');
    const linkClearanceInput = document.getElementById('linkClearance');
    const topologySelect = document.getElementById('optTopology');
    const hornDirectionSelect = document.getElementById('optHornDirection');
    // Horn direction is a C3 parameter; other topologies hide it and run outward.
    const syncHornDirectionField = () => {
        document.getElementById('optHornDirectionField').hidden = topologySelect.value !== 'c3_paired';
    };
    topologySelect.addEventListener('change', syncHornDirectionField);
    syncHornDirectionField();
    let currentOptimizer = null;
    let lastOutcome = null;
    let simulatorRun = null;
    let runSerial = 0;
    let runReferenceNote = '';
    const dashboard = createRunDashboard(document, { now });
    const fallbackButton = document.getElementById('runMainThreadFallback');
    const downloadFormat = document.getElementById('downloadFormat');
    const downloadButton = document.getElementById('downloadSelected');
    function offerFallback(available) {
        fallbackButton.hidden = !available;
        fallbackButton.disabled = !available;
    }
    offerFallback(false);
    let activeTab = 'optimize';
    const simCandidateSelect = document.getElementById('simCandidateSelect');
    // The reachability sweep yields to the page's animation frames between chunks.
    const simulatorController = createSimulatorController({ schedule: callback => typeof window.requestAnimationFrame === 'function'
        ? window.requestAnimationFrame(callback) : setTimeout(callback, 0) });
    const simulatorView = createSimulatorView({ document, window, controller: simulatorController,
        isActive: () => activeTab === 'simulate' });
    const geometryContainer = document.getElementById('simGeometryControls');
    const geometryControls = geometryContainer?.appendChild && geometryContainer?.replaceChildren
        ? createGeometryControls({ document, container: geometryContainer, controller: simulatorController })
        : null;
    const simulatorDiagnostics = mountSimulatorDiagnostics({ document, controller: simulatorController });
    function setTab(tab) {
        activeTab = tab;
        for (const [name, buttonId, panelId] of [['optimize', 'optimizeTab', 'optimizePanel'],
            ['simulate', 'simulateTab', 'simulatePanel']]) {
            const selected = name === tab;
            const button = document.getElementById(buttonId);
            button.classList.toggle('active', selected);
            button.setAttribute('aria-selected', String(selected));
            document.getElementById(panelId).hidden = !selected;
        }
        if (tab === 'simulate') simulatorView.render();
    }
    document.getElementById('optimizeTab').addEventListener('click', () => setTab('optimize'));
    document.getElementById('simulateTab').addEventListener('click', () => setTab('simulate'));
    const { populateRequirementsDefaults, readWorkspaceRanges, readHomeHeightBounds,
        readSamplingSettings, readCycleSampling, randomizeSeed } = createControls(document);
    const ratingControls = createServoRatingControls(document);
    function populate(parsed, preserveEdits = false) {
        populateRequirementsDefaults(parsed, preserveEdits);
        ratingControls.populate(parsed.normalized, preserveEdits);
    }
    installTooltips(document, window);
    // Candidate options of the current results, without any imported-reference entry.
    let candidateOptions = '';
    // The last successfully imported reference document, so the empty
    // "Imported reference" option can reload it after a candidate was shown.
    let importedReference = null;
    simCandidateSelect.addEventListener('change', () => {
        if (simCandidateSelect.value) {
            resultsView.select(simCandidateSelect.value);
        } else if (importedReference) {
            try { loadSimulatorLayout(structuredClone(importedReference), false); }
            catch (error) { showStatus(error.message, true); }
        }
    });
    function clearSimulatorSelection() {
        candidateOptions = '';
        importedReference = null;
        simCandidateSelect.innerHTML = '';
        simCandidateSelect.disabled = true;
    }
    function loadCandidate(candidate) {
        simCandidateSelect.value = String(candidate.layout.id);
        const settings = lastOutcome?.effective_settings ?? currentOptimizer?.effectiveSettings?.();
        simulatorController.loadLayout(candidate.layout, {
            source: { kind: 'candidate', candidateId: candidate.layout.id },
            options: simulatorOptions(settings, candidate.layout),
            // The run's requirement ranges, drawn as the workspace box.
            workspaceRanges: parseWorkspaceRanges(settings?.bounds, 'effective_settings.bounds'),
            // The run's payload and servo ratings, for the loads overlay.
            loadModel: loadModelFromSettings(settings),
        });
        // As in loadSimulatorLayout: only after the validating load.
        simulatorRun = lastOutcome;
        document.getElementById('simDownload').disabled = false;
    }
    const resultsView = createResultsView(document, (candidate) => {
        if (!currentOptimizer || currentOptimizer.running) return;
        currentOptimizer.selectCandidate(candidate.layout.id);
        setResultOutput(JSON.stringify({ run: lastOutcome, result: displayResult(candidate) }, null, 2));
        loadCandidate(candidate);
        setRunning(false);
    }, ChartClass);
    resultsView.clear();
    clearSimulatorSelection();

    function showStatus(message, isError = false) {
        statusEl.textContent = message;
        statusEl.classList.toggle('error', isError);
    }

    document.getElementById('loadSampleRequirements').addEventListener('click', async () => {
        try {
            const json = await loadDefaultRequirements();
            requirementsInput.value = json;
            populate(parseRequirements(json));
            showStatus('Sample requirements loaded.');
        } catch (error) {
            console.error(error);
            showStatus(error.message, true);
        }
    });

    document.getElementById('clearRequirements').addEventListener('click', () => {
        requirementsInput.value = '';
        setResultOutput('');
        currentOptimizer = null;
        lastOutcome = null;
        offerFallback(false);
        simulatorRun = null;
        simulatorController.clear();
        clearSimulatorSelection();
        document.getElementById('simDownload').disabled = true;
        resultsView.clear();
        dashboard.reset();
        setRunning(false);
        showStatus('Requirements cleared.');
    });

    document.getElementById('randomizeSeed').addEventListener('click', () => {
        try {
            showStatus(`Run seed set to ${randomizeSeed(window.crypto)}.`);
        } catch (error) { showStatus(error.message, true); }
    });

    document.getElementById('clearReferenceLayout').addEventListener('click', () => {
        referenceLayoutInput.value = '';
        document.getElementById('referenceLayoutFile').value = '';
        showStatus('Reference layout cleared.');
    });

    document.getElementById('referenceLayoutFile').addEventListener('change', async event => {
        const file = event.target?.files?.[0];
        if (!file) return;
        try {
            referenceLayoutInput.value = await file.text();
            showStatus(`Reference layout loaded: ${file.name}.`);
        } catch (error) {
            showStatus(`Could not read reference layout: ${error.message}`, true);
        }
    });

    function updateDownloadControls(running) {
        const selected = currentOptimizer?.getSelectedCandidate?.();
        const hasResult = Boolean(selected || currentOptimizer?.fitness?.length);
        downloadFormat.disabled = running || !hasResult;
        downloadButton.disabled = running || !hasResult
            || (downloadFormat.value !== 'json' && !canExportCad(selected));
    }

    function setRunning(running) {
        ratingControls.setDisabled(running);
        for (const id of ['runOptimization', 'loadSampleRequirements', 'clearRequirements',
            'optSampling', 'optCycleSampling', 'optSeed', 'randomizeSeed', 'optPopulation', 'optGenerations',
            'optObjectiveSet', 'optMutationRate', 'clearReferenceLayout', 'referenceLayoutFile',
            'saveLocalWorkspace', 'restoreLocalWorkspace', 'deleteLocalWorkspace']) {
            document.getElementById(id).disabled = running;
        }
        document.getElementById('cancelOptimization').disabled = !running;
        if (running) fallbackButton.disabled = true;
        updateDownloadControls(running);
    }

    downloadFormat.addEventListener('change', () => updateDownloadControls(Boolean(currentOptimizer?.running)));

    document.getElementById('cancelOptimization').addEventListener('click', () => {
        currentOptimizer?.stop();
        dashboard.cancelRequested(runSerial);
        showStatus('Cancelling optimization...');
    });

    function finalSnapshot() {
        if (currentOptimizer?.lastProgress) return currentOptimizer.lastProgress;
        if (!Number.isInteger(currentOptimizer?.completedEvaluations)) return null;
        return progressSnapshot(currentOptimizer, {
            completed: currentOptimizer.completedPoseWork ?? 0,
            generation: currentOptimizer.generation,
        }, dashboard.elapsedMs());
    }

    function presentOutcome(outcome, thisRun) {
        const pareto = currentOptimizer.pareto?.length ? currentOptimizer.pareto : currentOptimizer.fitness;
        const best = currentOptimizer.getSelectedCandidate?.()
            ?? selectBest(currentOptimizer.pareto, currentOptimizer.fitness);
        lastOutcome = { ...outcome, effective_settings: currentOptimizer.effectiveSettings?.() };
        resultsView.render(currentOptimizer.fitness, best?.layout.id);
        candidateOptions = currentOptimizer.fitness.map(item =>
            `<option value="${item.layout.id}">Candidate ${item.layout.id}</option>`).join('');
        simCandidateSelect.innerHTML = candidateOptions;
        simCandidateSelect.disabled = !currentOptimizer.fitness.length;
        setResultOutput(best ? JSON.stringify({ run: lastOutcome, result: displayResult(best) }, null, 2) : '');
        dashboard.finish(thisRun, { status: outcome.status, partialResults: outcome.partialResults,
            snapshot: finalSnapshot() });
        if (best) loadCandidate(best);
        if (outcome.status === 'cancelled') {
            showStatus(best ? 'Optimization cancelled. Showing partial results from the last completed population.' : 'Optimization cancelled before a population completed.');
        } else if (outcome.status === 'failed') {
            showStatus(`${outcome.error || 'Worker execution failed.'}${best ? ' Showing partial results from the last completed population.' : ''}`, true);
        } else {
            showStatus(`Optimization complete. Feasible coverage: ${best?.coverage ?? 0}%. Pareto front contains ${currentOptimizer.pareto.length || pareto.length} layouts. Coverage applies only to sampled poses and modeled constraints.${runReferenceNote}`);
        }
    }

    function reportProgress(thisRun, progress) {
        if (thisRun !== runSerial) return;
        const snapshot = Number.isFinite(progress.elapsedMs) && Number.isInteger(progress.completedCandidates)
            ? progress : progressSnapshot(currentOptimizer, progress, dashboard.elapsedMs());
        if (!dashboard.publish(thisRun, snapshot)) return;
        const completed = snapshot.actualCompletedPoseWork;
        const total = snapshot.budgetedPoseWork;
        showStatus(`Generation ${snapshot.generation}: ${completed.toLocaleString()} / ${total.toLocaleString()} pose evaluations (${total ? (100 * completed / total).toFixed(1) : '0.0'}%).`);
    }

    fallbackButton.addEventListener('click', async () => {
        if (!(currentOptimizer instanceof WorkerOptimizer) || !currentOptimizer.startupFailure || currentOptimizer.running) return;
        const thisRun = ++runSerial;
        offerFallback(false);
        currentOptimizer.onProgress = progress => reportProgress(thisRun, progress);
        const work = currentOptimizer.estimateWork();
        dashboard.start(thisRun, { candidates: work.evaluations,
            generations: currentOptimizer.generations, poseWork: work.totalPoses });
        showStatus('Running explicit main-thread fallback. The page yields between pose batches.');
        setRunning(true);
        try {
            const outcome = await currentOptimizer.startFallback();
            if (thisRun === runSerial) presentOutcome(outcome, thisRun);
        } catch (error) {
            if (thisRun === runSerial) {
                dashboard.finish(thisRun, { status: 'failed', snapshot: finalSnapshot() });
                showStatus(error.message, true);
            }
        } finally {
            if (thisRun === runSerial) setRunning(false);
        }
    });

    document.getElementById('runOptimization').addEventListener('click', async () => {
        if (currentOptimizer?.running) return;
        const thisRun = ++runSerial;
        offerFallback(false);
        try {
            const text = requirementsInput.value.trim();
            if (!text) {
                throw new Error('Provide requirements JSON before running the optimizer.');
            }
            const { normalized, workspace } = parseRequirements(text);
            populate({ normalized, workspace }, true);

            const generations = Number(document.getElementById('optGenerations').value);
            const populationSize = Number(document.getElementById('optPopulation').value);
            const objectiveSet = document.getElementById('optObjectiveSet').value;
            const mutationText = document.getElementById('optMutationRate').value.trim();
            if (!mutationText) throw new RangeError('mutationRate must be a finite probability in [0, 1].');
            const mutationRate = Number(mutationText);
            const ranges = readWorkspaceRanges();
            const { seed, sampling } = readSamplingSettings();
            const servoRatings = ratingControls.read();

            const OptimizerClass = Optimizer === DefaultOptimizer ? WorkerOptimizer : Optimizer;
            const options = {
                generations,
                populationSize,
                objectiveSet,
                mutationRate,
                ranges,
                topology: topologySelect.value || 'c3_paired',
                hornDirection: (topologySelect.value || 'c3_paired') === 'c3_paired' ? hornDirectionSelect.value || 'outward' : 'outward',
                referenceLayout: referenceLayoutInput.value.trim() || null,
                homeHeightBounds: readHomeHeightBounds(),
                sampling,
                cycleSampling: readCycleSampling(),
                seed,
                servoRatings,
                ballJointLimitDeg: Number(ballJointLimitInput.value),
                // An emptied field falls back to the requirements value.
                linkClearanceMm: linkClearanceInput.value.trim() === '' ? undefined : Number(linkClearanceInput.value),
                ballJointClamp: ballJointClampCheckbox.checked,
                onProgress: progress => reportProgress(thisRun, progress),
            };
            const optimizer = OptimizerClass === WorkerOptimizer
                ? new WorkerOptimizer(normalized, options, workerFactory ? { workerFactory } : {})
                : new OptimizerClass(normalized, options);
            // The pose-budget preflight depends only on the new instance. Run it before
            // replacing the previous run so a rejected budget leaves the previous results,
            // candidate list, simulator and dashboard on screen together.
            const work = optimizer.estimateWork();
            currentOptimizer = optimizer;
            if (currentOptimizer.topology) topologySelect.value = currentOptimizer.topology;
            if (currentOptimizer.hornDirection && currentOptimizer.topology === 'c3_paired') {
                hornDirectionSelect.value = currentOptimizer.hornDirection;
            }
            syncHornDirectionField();

            setResultOutput('');
            lastOutcome = null;
            simulatorRun = null;
            simulatorController.clear();
            clearSimulatorSelection();
            document.getElementById('simDownload').disabled = true;
            resultsView.clear();
            dashboard.start(thisRun, { candidates: work.evaluations,
                generations, poseWork: work.totalPoses });
            const reference = currentOptimizer.referenceDiagnostics;
            const migrationNote = currentOptimizer.referenceLayout?.migration?.note;
            const referenceNote = reference
                ? ` Reference: ${reference.boundsConflicts.length} search-bounds conflict(s); home pose ${reference.homePoseSatisfied ? 'valid' : 'invalid'}.${migrationNote ? ` ${migrationNote}` : ''}`
                : '';
            runReferenceNote = referenceNote;
            showStatus(`Optimization starting: ${work.totalPoses.toLocaleString()} pose evaluations.${referenceNote}`);
            setRunning(true);
            const outcome = await currentOptimizer.start();
            if (thisRun !== runSerial) return;

            presentOutcome(outcome, thisRun);
        } catch (error) {
            if (thisRun !== runSerial) return;
            dashboard.finish(thisRun, { status: 'failed',
                partialResults: currentOptimizer?.fitness?.length > 0, snapshot: finalSnapshot() });
            if (error.startupFailure) {
                showStatus(`Worker startup failed: ${error.message} Select “Run on main thread” to continue.`, true);
                offerFallback(true);
            } else {
                console.error(error);
                showStatus(error.message, true);
            }
        } finally {
            if (thisRun === runSerial) setRunning(false);
        }
    });

    function exportLayout() {
        try {
            if (!currentOptimizer) {
                showStatus('Run the optimization before exporting.', true);
                return;
            }
            const data = currentOptimizer.exportBest();
            if (data !== undefined) downloadFile(data, 'optimized_layout.json', 'application/json', document);
        } catch (error) {
            console.error(error);
            showStatus(error.message, true);
        }
    }

    function simulatorJSON() {
        const state = simulatorController.getState();
        if (!state.layout) throw new Error('Load a layout before exporting simulator state.');
        return JSON.stringify({ ...layoutToJSON(state.layout), run: simulatorRun,
            simulator: { source: state.source, requested: state.requested, accepted: state.accepted,
                options: state.options, animation: state.animation, markers: state.markers,
                tracesEnabled: state.tracesEnabled, overlays: state.overlays, reachability: state.reachability,
                workspaceRanges: workspaceRangesToJSON(state.workspaceRanges), loadModel: state.loadModel, trace: state.trace,
                camera: simulatorView.getCamera(), pointerMode: document.getElementById('simPointerMode').value,
                inputFrame: simulatorView.getInputFrame() } }, null, 2);
    }
    document.getElementById('simUseReference').addEventListener('click', () => {
        try {
            referenceLayoutInput.value = simulatorJSON();
            setTab('optimize');
            showStatus('Simulator geometry is ready as the optimizer reference.');
        } catch (error) { showStatus(error.message, true); }
    });
    document.getElementById('simDownload').addEventListener('click', () => {
        try { downloadFile(simulatorJSON(), 'stewart_simulator.json', 'application/json', document); }
        catch (error) { showStatus(error.message, true); }
    });
    // Everything that can reject a file happens here, before any state changes.
    function prepareSimulatorLoad(parsed) {
        const { layout, sourceRun } = importLayout(parsed);
        layout.id = parsed.id ?? parsed.layout?.id ?? parsed.result?.layout?.id ?? null;
        const saved = parseSimulatorSnapshot(parsed.simulator, layout, simulatorOptions(sourceRun?.effective_settings, layout),
            sourceRun?.effective_settings?.bounds, sourceRun?.effective_settings);
        return { parsed, layout, sourceRun, saved };
    }
    function applySimulatorLoad({ parsed, layout, sourceRun, saved }, activate = true) {
        simulatorController.loadLayout(layout, { source: { kind: 'import', candidateId: layout.id ?? null },
            options: saved.options, workspaceRanges: saved.workspaceRanges, loadModel: saved.loadModel });
        // Only after the validating load, so a rejected file never pairs its run
        // metadata with the candidate that stays loaded.
        simulatorRun = sourceRun;
        importedReference = structuredClone(parsed);
        simCandidateSelect.innerHTML = `<option value="">Imported reference</option>${candidateOptions}`;
        simCandidateSelect.value = '';
        // The select holds at least the imported entry, even without a run.
        simCandidateSelect.disabled = false;
        if (saved.accepted) simulatorController.requestPose(saved.accepted, { source: 'replay' });
        if (saved.requested) simulatorController.requestPose(saved.requested, { source: 'replay' });
        if (saved.camera) simulatorView.setCamera(saved.camera);
        if (saved.animation) {
            const { pattern, speed } = saved.animation;
            document.getElementById('simPattern').value = pattern;
            document.getElementById('simSpeed').value = String(speed);
            simulatorController.setAnimation(pattern, false, { speed });
        }
        if (saved.pointerMode) document.getElementById('simPointerMode').value = saved.pointerMode;
        if (saved.inputFrame) simulatorView.setInputFrame(saved.inputFrame);
        if (saved.markers !== null) simulatorController.setMarkers(saved.markers);
        if (saved.tracesEnabled !== null) simulatorController.setTraces(saved.tracesEnabled);
        if (saved.reachability) simulatorController.setReachabilityCloud(saved.reachability);
        if (saved.overlays) simulatorController.setOverlays(saved.overlays);
        document.getElementById('simDownload').disabled = false;
        if (activate) setTab('simulate');
    }
    function loadSimulatorLayout(parsed, activate = true) {
        applySimulatorLoad(prepareSimulatorLoad(parsed), activate);
    }
    document.getElementById('simLoadReference').addEventListener('click', () => {
        try {
            const raw = referenceLayoutInput.value.trim();
            if (!raw) throw new Error('Provide reference layout JSON in Optimize first.');
            loadSimulatorLayout(parseLayoutJSON(raw));
        } catch (error) { showStatus(error.message, true); setTab('optimize'); }
    });

    function restoreLocalWorkspace(raw) {
        const saved = parseLocalWorkspace(raw);
        // Validate the saved simulator document before any input or layout changes,
        // so a rejected save leaves the seed, layout and candidate select as they were.
        const prepared = saved.simulator ? prepareSimulatorLoad(saved.simulator) : null;
        currentOptimizer = null;
        lastOutcome = null;
        simulatorRun = null;
        offerFallback(false);
        simulatorController.clear();
        clearSimulatorSelection();
        document.getElementById('simDownload').disabled = true;
        resultsView.clear();
        setResultOutput('');
        dashboard.reset();
        if (saved.inputs.requirementsInput.trim()) {
            try { populate(parseRequirements(saved.inputs.requirementsInput)); }
            catch { /* Preserve unfinished requirements text and saved control values. */ }
        }
        applyLocalWorkspace(document, saved);
        syncHornDirectionField();
        if (prepared) applySimulatorLoad(prepared, false);
        setRunning(false);
        showStatus(saved.simulator
            ? 'Local workspace restored. The saved layout is ready in Simulate; rerun optimization for candidate results.'
            : 'Local workspace restored. Run optimization to regenerate results.');
    }

    document.getElementById('saveLocalWorkspace').addEventListener('click', () => {
        try {
            const simulator = simulatorController.getState().layout ? JSON.parse(simulatorJSON()) : null;
            window.localStorage.setItem(LOCAL_WORKSPACE_KEY,
                JSON.stringify(captureLocalWorkspace(document, simulator)));
            showStatus('Workspace saved in this browser.');
        } catch (error) { showStatus(`Could not save in this browser: ${error.message}`, true); }
    });
    document.getElementById('restoreLocalWorkspace').addEventListener('click', () => {
        try {
            const raw = window.localStorage.getItem(LOCAL_WORKSPACE_KEY);
            if (!raw) throw new Error('No browser save exists.');
            restoreLocalWorkspace(raw);
        } catch (error) { showStatus(`Could not restore browser save: ${error.message}`, true); }
    });
    document.getElementById('deleteLocalWorkspace').addEventListener('click', () => {
        try {
            window.localStorage.removeItem(LOCAL_WORKSPACE_KEY);
            showStatus('Browser save deleted.');
        } catch (error) { showStatus(`Could not delete browser save: ${error.message}`, true); }
    });

    function exportCad(format) {
        try {
            const selected = currentOptimizer?.getSelectedCandidate?.();
            if (!selected) throw new Error('Select a completed candidate before CAD export.');
            const run = JSON.parse(currentOptimizer.exportBest()).run;
            const skeleton = buildConstructionSkeleton(selected, run);
            if (format === 'fusion') {
                downloadFile(skeletonToFusionScript(skeleton), 'stewart_construction.py', 'text/x-python', document);
            } else {
                downloadFile(skeletonToCSV(skeleton), 'stewart_coordinates.csv', 'text/csv', document);
            }
            if (skeleton.diagnostic) showStatus(`CAD construction geometry exported for diagnostic candidate ${skeleton.candidateId}; failed categories: ${skeleton.failedCategories.join(', ')}.`);
        } catch (error) {
            showStatus(error.message, true);
        }
    }

    downloadButton.addEventListener('click', () => {
        if (downloadFormat.value === 'json') exportLayout();
        else if (downloadFormat.value === 'fusion' || downloadFormat.value === 'csv') exportCad(downloadFormat.value);
    });

    const ready = loadDefaultRequirements()
        .then((json) => {
            requirementsInput.value = json;
            populate(parseRequirements(json));
            showStatus('Sample requirements loaded. Adjust parameters and run the optimizer.');
        })
        .catch((error) => {
            console.error(error);
            showStatus(error.message, true);
        })
        .then(() => {
            try {
                const saved = window.localStorage?.getItem(LOCAL_WORKSPACE_KEY);
                if (!saved) return;
                // The user may have started a run before the sample fetch settled.
                // Restoring now would null the optimizer under the active run.
                if (currentOptimizer?.running) {
                    showStatus('Optimization running; the browser save was not restored automatically. Use Restore browser save after the run finishes.');
                    return;
                }
                restoreLocalWorkspace(saved);
            } catch (error) {
                showStatus(`Could not restore browser save: ${error.message}`, true);
            }
        });
    return { ready, simulatorController, simulatorView, geometryControls, simulatorDiagnostics, setTab };
}
