// Compactness: the radius (mm) of the smallest vertical cylinder about the Z
// axis that holds every base anchor's horn reach (anchor radius plus horn
// length, wherever the horn points) and every platform anchor. It is pure
// geometry, so it costs no pose evaluations, and it is conservative: a horn
// that never swings outward still counts at full length.
export function layoutFootprint(layout) {
  const radius = ([x, y]) => Math.hypot(x, y);
  const base = Math.max(...layout.baseAnchors.map(anchor => radius(anchor) + layout.hornLength));
  const platform = Math.max(...layout.platformAnchors.map(radius));
  const footprint = Math.max(base, platform);
  return Number.isFinite(footprint) ? footprint : null;
}
