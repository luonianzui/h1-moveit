def observation_phase(command_phase,retrieval_closing):
    if retrieval_closing is True and command_phase in ('left_grasp','right_grasp'):
        return command_phase.split('_')[0]+'_retrieval_grasp'
    return command_phase
