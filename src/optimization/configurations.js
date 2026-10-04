import { isPassing } from '../io/results.js';
import { C3_HORN_DIRECTIONS, C3_LEG_PAIRINGS, DEFAULT_C3_HORN_DIRECTION, DEFAULT_C3_LEG_PAIRING,
  c3HornDirection, c3LegPairing } from './topology.js';

// Half of a multi-configuration run's population is reserved, split evenly,
// so no searched configuration can be crowded out before it has been refined.
export const CONFIGURATION_SHARE = 0.5;

// The C3 configurations a run searches: every combination of the horn
// directions and leg pairings its modes allow. Other topologies have none.
export function searchedConfigurations({ topology, hornDirection, legPairing }) {
  if (topology !== 'c3_paired') return [];
  const horns = hornDirection === 'both' ? C3_HORN_DIRECTIONS : [hornDirection ?? DEFAULT_C3_HORN_DIRECTION];
  const pairings = legPairing === 'both' ? C3_LEG_PAIRINGS : [legPairing ?? DEFAULT_C3_LEG_PAIRING];
  return horns.flatMap(direction => pairings.map(pairing => ({ hornDirection: direction, legPairing: pairing })));
}

export function layoutConfiguration(layout) {
  if (layout?.topology !== 'c3_paired') return null;
  const parameters = layout.topologyParameters ?? layout.topology_parameters;
  return { hornDirection: c3HornDirection(parameters), legPairing: c3LegPairing(parameters) };
}

export const configurationKey = configuration => configuration
  ? `${configuration.hornDirection}/${configuration.legPairing}` : null;

const keyOf = evaluation => configurationKey(layoutConfiguration(evaluation.layout));

// Guarantees each searched configuration a share of the survivors. `ranked`
// is the merged population best first (front order, then crowding); a
// configuration below its quota takes its best unselected members from it,
// each replacing the lowest-ranked survivor of a configuration above quota.
// Deterministic, so it consumes no random draws. `protectedEvaluation` (the
// reference) is never replaced.
export function retainConfigurations(selected, ranked, configurations, populationSize, protectedEvaluation = null) {
  if (configurations.length < 2) return selected;
  const quota = Math.max(1, Math.floor(populationSize * CONFIGURATION_SHARE / configurations.length));
  const survivors = selected.slice();
  const counts = new Map(configurations.map(configuration => [configurationKey(configuration), 0]));
  for (const evaluation of survivors) counts.set(keyOf(evaluation), (counts.get(keyOf(evaluation)) ?? 0) + 1);
  for (const configuration of configurations) {
    const key = configurationKey(configuration);
    const candidates = ranked.filter(evaluation => keyOf(evaluation) === key && !survivors.includes(evaluation));
    while (counts.get(key) < quota && candidates.length) {
      let victim = -1;
      for (let i = survivors.length - 1; i >= 0; i--) {
        const other = keyOf(survivors[i]);
        if (survivors[i] !== protectedEvaluation && other !== key && (counts.get(other) ?? 0) > quota) { victim = i; break; }
      }
      if (victim < 0) break;
      counts.set(keyOf(survivors[victim]), counts.get(keyOf(survivors[victim])) - 1);
      survivors[victim] = candidates.shift();
      counts.set(key, counts.get(key) + 1);
    }
  }
  return survivors;
}

// One row per searched configuration: how many layouts the population keeps,
// how many pass every constraint, and the best feasible coverage among them.
export function configurationSummary(evaluations, configurations) {
  return configurations.map(configuration => {
    const key = configurationKey(configuration);
    const members = evaluations.filter(evaluation => keyOf(evaluation) === key);
    const feasible = members.filter(isPassing);
    const coverage = members.map(evaluation => evaluation.coverage).filter(Number.isFinite);
    return { configuration: key, ...configuration, retained: members.length, feasible: feasible.length,
      bestCoverage: coverage.length ? Math.max(...coverage) : null };
  });
}

export function describeConfigurationSummary(summary) {
  return summary.map(row => `${row.configuration} ${row.retained} kept, ${row.feasible} feasible`
    + (row.bestCoverage == null ? '' : `, best ${row.bestCoverage}%`)).join('; ');
}
