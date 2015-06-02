function ininfluentJoints = srv_identify_ininfluent_joint_set(attachIndices, dofsPerLimb)
    ininfluentJoints = [0, sum(dofsPerLimb(1:end))];
    if ~isempty(attachIndices),
        linkNo = attachIndices(1);
        limbNo = attachIndices(2);
        ininfluentJoints = [sum(dofsPerLimb(1:limbNo-1)), ...
                                dofsPerLimb(limbNo)-linkNo+sum(dofsPerLimb(limbNo+1:end))];
    end