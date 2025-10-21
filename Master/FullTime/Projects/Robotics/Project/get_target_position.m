function pos = get_target_position(label)
% Returns the [x, y, z] position for a given target label (e.g., 'D3')
% All positions are based on a 3x3 grid layout with 50 mm spacing

    switch upper(label)
        % --- Top-left grid (A–C) ---
        case 'A1', pos = [75, -75, 50]';
        case 'B1', pos = [50, -75, 50]';
        case 'C1', pos = [25, -75, 50]';
        case 'A2', pos = [75, -50, 50]';
        case 'B2', pos = [50, -50, 50]';
        case 'C2', pos = [25, -50, 50]';
        case 'A3', pos = [75, -25, 50]';
        case 'B3', pos = [50, -25, 50]';
        case 'C3', pos = [25, -25, 50]';

        % --- Bottom-left grid (D–F) ---
        case 'D1', pos = [75,  25, 50]';
        case 'E1', pos = [50,  25, 50]';
        case 'F1', pos = [25,  25, 50]';
        case 'D2', pos = [75,  50, 50]';
        case 'E2', pos = [50,  50, 50]';
        case 'F2', pos = [25,  50, 50]';
        case 'D3', pos = [75,  75, 50]';
        case 'E3', pos = [50,  75, 50]';
        case 'F3', pos = [25,  75, 50]';

        % --- Top-right grid (G–I) ---
        case 'G1', pos = [-25, -75, 50]';
        case 'H1', pos = [-50, -75, 50]';
        case 'I1', pos = [-75, -75, 50]';
        case 'G2', pos = [-25, -50, 50]';
        case 'H2', pos = [-50, -50, 50]';
        case 'I2', pos = [-75, -50, 50]';
        case 'G3', pos = [-25, -25, 50]';
        case 'H3', pos = [-50, -25, 50]';
        case 'I3', pos = [-75, -25, 50]';

        % --- Bottom-right grid (J–L) ---
        case 'J1', pos = [-25,  25, 50]';
        case 'K1', pos = [-50,  25, 50]';
        case 'L1', pos = [-75,  25, 50]';
        case 'J2', pos = [-25,  50, 50]';
        case 'K2', pos = [-50,  50, 50]';
        case 'L2', pos = [-75,  50, 50]';
        case 'J3', pos = [-25,  75, 50]';
        case 'K3', pos = [-50,  75, 50]';
        case 'L3', pos = [-75,  75, 50]';
        
        otherwise
            error('Invalid label: %s. Use a label like A1, D3, K2, etc.', label);
    end
    pos = pos + [0,0,356.521686]';
end
