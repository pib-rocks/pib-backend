def test_update_description_writes_soul_file(tmp_path, monkeypatch, app_ctx, make_personality):
    monkeypatch.setattr("service.soul_service.HERMES_HOME", str(tmp_path))
    from service import personality_service, soul_service

    p = make_personality(description="alt")
    personality_service.update_personality(p.personality_id, {"description": "neu"})

    assert soul_service.read_soul(p.personality_id) == "neu"
