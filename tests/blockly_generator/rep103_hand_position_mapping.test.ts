import { execFileSync } from "node:child_process";

/**
 * Same axis permutation as pib_sdk.from_rep103 / to_rep103 (REP-103 body frame vs model frame).
 * Duplicated here so generator tests stay meaningful when pib-sdk is not installed in CI.
 */
function fromRep103(x: number, y: number, z: number): [number, number, number] {
  return [y, -x, z];
}

describe("REP-103 hand position axis mapping", () => {
  it("maps 400 mm forward in REP-103 to model frame (0, -400, 0)", () => {
    expect(fromRep103(400, 0, 0)).toEqual([0, -400, 0]);
  });

  it("matches pib_sdk.from_rep103 when pib-sdk is installed", () => {
    const script = `
import sys
try:
    from pib_sdk import from_rep103
except ImportError:
    sys.exit(2)
result = list(from_rep103([400, 0, 0]))
assert result == [0, -400, 0], result
`;
    try {
      execFileSync("python3", ["-c", script], {
        encoding: "utf-8",
        stdio: ["pipe", "pipe", "pipe"],
      });
    } catch (error: unknown) {
      const status =
        error && typeof error === "object" && "status" in error
          ? (error as { status?: number }).status
          : undefined;
      if (status === 2) {
        return;
      }
      throw error;
    }
  });
});
