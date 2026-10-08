"""Temporary red-proof for PR-1932: must fail on purpose. Removed in the follow-up commit."""


def test_pr1932_temporary_red_proof() -> None:
    assert False, "deliberate failure: proves the unit job fails when a unit test fails"
