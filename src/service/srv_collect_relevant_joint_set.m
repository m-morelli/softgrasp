function cRelevantJointConfig = srv_collect_relevant_joint_set(q, ininfluentJoints, attachIndices)
    cRelevantJointConfig = zeros(1,0);
    if ~isempty(attachIndices),
        cRelevantJointConfig = q(ininfluentJoints(1)+1:ininfluentJoints(1)+attachIndices(1));
    end
