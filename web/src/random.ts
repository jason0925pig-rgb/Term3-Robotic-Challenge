export function makeRng(seed: number): () => number {
  let s = seed >>> 0;

  return () => {
    s = (1664525 * s + 1013904223) >>> 0;
    return s / 0xffffffff;
  };
}
