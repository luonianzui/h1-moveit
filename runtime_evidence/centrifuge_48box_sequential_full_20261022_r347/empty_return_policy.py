def skip_verified_empty_return(is_last_payload, phase, holding, held_payload,
                               table_verified, retreat_complete):
    return (not is_last_payload and phase == 'return' and holding is None
            and held_payload is None and table_verified is True
            and retreat_complete is True)

