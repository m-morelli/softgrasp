function [rr] = srv_a2noa(a_)

    a_ = a_(:);
    if (norm(a_) == 0),
        rr = eye(3);
        return;
    end

    a = unit(a_);
    mm_ = eye(3) - a*a';
    mm = [];
    for i = 1:3,
        kk = mm_(:,i);
        if norm(kk) > 1e-8,
            kk = unit(kk);
        end
        mm = [mm, kk];
    end
    [mmef, iidx] = rref(mm);
    nn = orth([unit(mm(:,iidx(1))), unit(mm(:,iidx(2)))]);
    rr = [nn(:,1), nn(:,2), a];
    if max(abs(rr'*rr - eye(3))) > 1e-14,
        error('GRASP:classProps:UnableToCompute', ...
            'unable to create the contact frame from Cp and Cn');
    end
    if abs(det(rr) - 1) > 1e-14,
        rr(:,1:2) = rr(:,2:-1:1);
    end
    if abs(det(rr) - 1) > 1e-14,
        error('GRASP:classProps:UnableToCompute', ...
            'unable to create the contact frame from Cp and Cn');
    end