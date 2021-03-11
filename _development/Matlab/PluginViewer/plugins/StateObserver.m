function StateObserver(area, globalConfig, pluginConfig)

    edgef = sprintf('%s%s.log',globalConfig.logPath, pluginConfig.sensorName);

    if exist(edgef, 'file')
        
        [edge, outcome] = stubbornLoad(edgef);
        
        if outcome == 1
            isQuaternion = false;
            
            subplot('Position', squeezeArea(area,0.05))
            grid on
            hold on
            x_axis = 'Time [s]';
            plot_legend = {};
            if strcmp(pluginConfig.stateComponent, 'x')
                range = 4:6;
                state_name = 'Position';
                y_axis = 'Position [m]';
                plot_legend = {'x', 'y', 'z'};
            elseif strcmp(pluginConfig.stateComponent, 'q')
                range = 7:10;
                isQuaternion = true;
                state_name = 'Orientation';
                y_axis = 'Orientation [rad]';
                plot_legend = {'roll', 'pitch', 'yaw'};
            elseif strcmp(pluginConfig.stateComponent, 'v')
                range = 11:13;
                state_name = 'Velocity';
                y_axis = 'Velocity [m.s^{-1}]';
                plot_legend = {'v_x', 'v_y', 'v_z'};
            elseif strcmp(pluginConfig.stateComponent, 'w')
                range = 14:16;
                state_name = 'Angular Velocity';
                y_axis = 'Angular Velocity [rad.s^{-1}]';
                plot_legend = {'{\omega}_x', '{\omega}_y', '{\omega}_z'};
            elseif strcmp(pluginConfig.stateComponent, 'a')
                range = 17:19;
                state_name = 'Acceleration';
                y_axis = 'Acceleration [m.s^{-2}]';
                plot_legend = {'a_x', 'a_y', 'a_z'};
            elseif strcmp(pluginConfig.stateComponent, 'alpha')
                range = 20:22;
                y_axis = '';
            end

            if (isfield(pluginConfig', 'referenceFile') && exist(pluginConfig.referenceFile, 'file'))
                if strcmp(pluginConfig.stateComponent, 'x')
                    range_gt = 2:4;
                elseif strcmp(pluginConfig.stateComponent, 'q')
                    range_gt = 5:8;
                    isQuaternion = true;
                elseif strcmp(pluginConfig.stateComponent, 'v')
                    range_gt = 9:11;
                elseif strcmp(pluginConfig.stateComponent, 'w')
                    range_gt = 12:14;
                elseif strcmp(pluginConfig.stateComponent, 'a')
                    range_gt = 15:17;
                elseif strcmp(pluginConfig.stateComponent, 'alpha')
                    range_gt = 18:20;
                end
                
                [ground_truth, ~] = stubbornLoad(pluginConfig.referenceFile);

                % get values of groundtruth when at the same timesteps than the measurements
                reference = interp1(ground_truth(:,1), ground_truth(:,range_gt), edge(:,1));

                if isQuaternion
                    quat_rot_error = quatprod_vec(quatinv(edge(:,range)), reference);
                    state_error = rad2deg(quat2euler(quat_rot_error));
                else
                    state_error = edge(:,range) - reference;
                end
                plot(edge(:,1) - edge(1,1), state_error)
                title(sprintf('%s: error on estimated %s',pluginConfig.sensorName, state_name), 'interpreter', 'none');
            else
                if isQuaternion
                    rpy_from_quat = rad2deg(quat2euler(edge(:,range)));
                    plot(edge(:,1) - edge(1,1),rpy_from_quat)
                else
                    plot(edge(:,1) - edge(1,1),edge(:,range))
                end
                title(sprintf('%s: estimated %s',pluginConfig.sensorName, state_name), 'interpreter', 'none');
            end

            axis tight

            xlabel(x_axis);
            ylabel([y_axis]);
            legend(plot_legend);
            hold off
        end    
    end

end

