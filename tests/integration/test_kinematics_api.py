"""Contract tests for POST /kinematics/inverse (PR-2180).

The endpoint is a thin wrapper around pib-sdk's solver, so these tests run the real solver and
check the answer against pib-sdk's own forward kinematics. They are skipped on a host without
pib-sdk (the CI job and the API image install the pinned version).
"""

from __future__ import annotations

import math
import time

import pytest

pib_sdk = pytest.importorskip("pib_sdk")
from pib_sdk import ArmKinematics

ZERO_RIGHT_MM = (-609.1, 106.2, 731.7)


@pytest.fixture()
def client():
    from app.app import app

    app.config.update(TESTING=True)
    with app.test_client() as test_client:
        yield test_client


def _post(client, body):
    return client.post("/kinematics/inverse", json=body)


class TestRequestValidation:
    def test_missing_target_is_rejected(self, client):
        response = _post(client, {"side": "right"})
        assert response.status_code == 400
        assert response.get_json()["error"] == "invalid request"

    def test_unknown_side_is_rejected(self, client):
        response = _post(client, {"side": "middle", "xyzM": [0, 0, 1]})
        assert response.status_code == 400

    def test_two_component_target_is_rejected(self, client):
        response = _post(client, {"side": "right", "xyzM": [0.1, 0.2]})
        assert response.status_code == 400


class TestAnswer:
    def test_reachable_target_returns_joint_angles_that_hit_it(self, client):
        xyz_m = [c / 1000.0 for c in ZERO_RIGHT_MM]
        response = _post(client, {"side": "right", "xyzM": xyz_m})
        assert response.status_code == 200
        body = response.get_json()

        assert body["reach"]["ok"] is True
        assert body["reach"]["distanceMm"] == 0
        assert body["reach"]["blockedBy"] is None
        assert len(body["jointsRad"]) == 6
        assert body["jointNames"]
        assert body["elapsedMs"] >= 0

        chain = ArmKinematics("right")
        pose = chain.forward(body["jointsRad"])
        error_mm = math.dist(list(pose.translation), list(ZERO_RIGHT_MM))
        assert error_mm < 1.0

    def test_unreachable_target_is_an_answer_not_an_error(self, client):
        response = _post(client, {"side": "right", "xyzM": [3.0, 0.0, 1.0]})
        assert response.status_code == 200
        body = response.get_json()
        assert body["reach"]["ok"] is False
        assert body["reach"]["blockedBy"] in {"workspace", "behind_body"}
        assert body["reach"]["distanceMm"] > 10.0
        # Nothing on the ray is reachable: no invented pose.
        assert body["jointsRad"] is None

    def test_target_beyond_reach_stops_at_the_edge(self, client):
        # Along the arm's own direction, 1.2 m out - past the 1.04 m reach, but the ray crosses
        # the workspace, so the answer is the boundary posture plus the remaining distance.
        direction = [c / 956.0 for c in ZERO_RIGHT_MM]
        target = [c * 1200.0 / 1000.0 for c in direction]
        response = _post(client, {"side": "right", "xyzM": target})
        assert response.status_code == 200
        body = response.get_json()
        assert body["reach"]["ok"] is False
        assert len(body["jointsRad"]) == 6
        assert 10.0 < body["reach"]["distanceMm"] < 400.0

    def test_seed_is_used_and_still_reaches_the_target(self, client):
        chain = ArmKinematics("right")
        seed = list(chain.forward([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]).translation)
        response = _post(
            client,
            {
                "side": "right",
                "xyzM": [c / 1000.0 for c in seed],
                "seedRad": [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            },
        )
        assert response.status_code == 200
        assert response.get_json()["reach"]["ok"] is True

    def test_a_hard_target_answers_inside_the_drag_budget(self, client):
        # This target used to converge never and burn 4.5 s inside pib-sdk's 50x200 search, long
        # after the console had given up. With the drag budget it has to answer quickly.
        started = time.perf_counter()
        response = _post(
            client,
            {
                "side": "right",
                "xyzM": [-0.66, 0.10, 0.79],
                "restarts": 2,
                "maxIterations": 60,
            },
        )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        assert response.status_code == 200
        assert elapsed_ms < 500.0, elapsed_ms
        body = response.get_json()
        assert body["reach"]["ok"] is False
        assert body["reach"]["blockedBy"] in {
            "workspace",
            "behind_body",
            "joint_limit",
            "convergence",
        }

    def test_thirty_solves_stay_inside_the_drag_budget(self, client):
        xyz_m = [c / 1000.0 for c in ZERO_RIGHT_MM]
        started = time.perf_counter()
        for _ in range(30):
            assert _post(client, {"side": "right", "xyzM": xyz_m}).status_code == 200
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        # Generous: the drag loop's own budget is 120 ms per command, this only catches a
        # pathological regression (a solve is 1.5-9 ms on its own, see PR-2056).
        assert elapsed_ms < 1500.0
