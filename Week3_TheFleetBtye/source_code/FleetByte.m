%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CSC C85 - Fundamentals of Robotics and Automated Systems
% UTSC - Fall 2021
%
% Starter code (c) F. Estrada, August 2021
% 
% Sensors and Signal Processing
%
%  You may have heard there are all kinds of plans to
% send humans to Mars. Eventually, some think we may
% establish long-term habitats on the martian surface
% occupied for long periods by 'martians'.
%
%  One of the challenges of maintaining a long term
% presence on Mars is that martian gravity is much
% weaker than Earth's, and though it's still much
% better than long-term living in space, martian
% explorers would need to keep a serious exercise
% program in order to prevent physical deterioration.
%
%  To help with this task, we've developed the
% FleetByte(tm). A device worn on a person's wrist
% that keeps track of their exercise. It's similar to
% devices you may be familiar with (or indeed which
% you may be wearing). Our goal here is to design
% the sensor and signal processing software that
% will convert the raw measurements provided by
% sensors in the device into accurate estimates
% of what the human wearing it is doing.
%
% Your task is to:
%
% a) Understand the different sensors available,
%    the values they report, and their noise profile.
% b) Implement suitable noise reduction and estimation
%    routines so as to obtain estimates of state variables
%    that are as close as possible to their actual values.
% c) Apply the ideas we discussed in lecture: Noise
%    filtering, consistency, and information redundancy
%    in order to obtain good estimates for state variables.
%
% []=FleetByte(secs, debug)
%
%   secs - number of (virtual) seconds to run the simulation for.
%          Each call to Sim1() returns sensor readings for 1 sec,
%          so this is in effect the numbe or rounds of simulation
%          you want. 
%
%   map - Select map (1 or 2), each is a crop from the global Mars
%         elevation map from NASA - image in public domain. Note
%         that motion on the map is *not to scale*, the map corrsponds
%         to a huge area on Mars, and I want to show motion on this
%         map. So we will pretend it corresponds to an area roughly
%         .5 x .5 Km in size.
%
%  debug - If set to 1, this script will plot the returned hearrate
%          sensor output (so you can see what it looks like and think
%          about how to get a heartrate out of it), and print out
%          the sensor readings returned by Sim1(). You can add your
%          own debug/testing output as well.
%               
% - delta_t - maximum change in rover direction per unit of time, in radians
%
%  On Board Sensors:
%
%  MPS - Martian Positioning System - reports 3D position anywhere on Mars
%        to within a small displacement from actual location. Like its
%        Earthly cousin, MPS has an expected location error. For 
%        a typical wearable device, on Earth, location error is
%        within 5m of the actual location 
%        (https://www.gps.gov/systems/gps/performance/accuracy/)
%        Our FleetByte has a similar receiver, but due to the lower
%        density of Martian atmosphere, distortion due to armospheric
%        effects is lower. Under open sky this means a typical location
%        accuracy of less than 1.5m.
%
%        Note: On Mars we don't have to worry about buildings. On Earth things
%          are more difficult since buildings reflect GPS signals leading
%          to increased error in position estimates.
%
%  Heart Rate Sensor (HRS) - This one is interesting. Modern wearable 
%        HR monitors typically use light reflection from 
%        arterial blood to determine the heart rate - the
%        pulsing blood creates a periodic waveform in the 
%        reflected light. Issues with noise, low signal-to-noise
%        ratio, and effects due to skin colour, thickness, and
%        even ambient light combine to produce a fairly noisy
%        signal. The HR sensor will return an array consisting
%        of the signal measured over the last 10 seconds, from
%        which you will estimate the actual heartrate.
%        If you're very curious, this manufacturer has a
%        very thorough description of how their sensor works and
%        the different technical issues involved in computing a
%        heartrate from it ** YOU ARE NOT EXPECTED TO READ 
%        THROUGH AND IMPLEMENT THIS, IT'S THERE IN CASE YOU 
%        WANT TO LEARN MORE **
%        https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html#product-details
%
%  Rate gyro (RG) - A fairly standard rate gyro, returns the measured
%              change in angle for the direction of motion (i.e.
%              tells you by how many radians this direction changed
%              in between readings). 
%
%              Somewhat noisy, but this assuming the user doesn't
%              move their arms in weird directions while running
%              it won't be affected by periodic arm motions.
%
%  ** The simulation returns to you the values measured by each
%  ** of these sensors at 1 second intervals. It's up to you to
%  ** decide how best to use/combine/denoise/filter/manipulate
%  ** the sensor readings to produce a good estimate of the actual
%  ** values of the relevant variables (which are also returned
%  ** by the simulation for the purpose of evaluating your
%  ** estimates' accuracy - needless to say, you can't use these
%  ** in any way, shape, or form, to accompish your task.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function []=FleetByte(secs, map, debug)

pkg load image;             %%% UNCOMMENT THIS FOR OCTAVE - Octave is doofus and requires this line... arghh!

close all;
%%%%%%%%%% YOU CAN ADD ANY VARIABLES YOU MAY NEED BETWEEN THIS LINE... %%%%%%%%%%%%%%%%%

prev_xyz = [128 128 0.5];
distance = 0;
direction_vector_2D = [0 1];
prev_vel = 5;
pred_xy = [128 128];
pred_di = [0 1];
delta_t = pi/4;
prev_rg = 0;
last_five = zeros(1,5);
hist_theta = [];
hist_di_error = [];
prev_hr = 70;
hist_vel = [];

%%%%%%%%%% ... AND THIS LINE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

idx=1;
while(idx<=secs)               %% Main simulation loop

 [MPS,HRS,Rg]=Sim1(map);       % Simulates 1-second of running and returns the sensor readings
                         
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 % TO DO:
 %  Sim1() returns noisy readings for global position (x,y,z), a heart-rate
 %    sensor signal array for the last 10 seconds, and a value for the rate
 %    gyro (the amount of rotation in radians by which the running direction
 %    changed over the last second).
 %
 %    In the space below, write code to:
 %
 %    - Estimate as closely as possible the actual 3D position of the jogger
 %
 %    - Compute the current hear-rate (this will require some thought, make
 %      sure to look closely at the plot of HRS, and think of ways in which
 %      you can determine the heart rate from this). Remember the data 
 %      in the plot corresponds to the last 10 seconds. And, just FYI, it's
 %      based on what the actual data returned from a typical wrist-worn
 %      heart rate monitor returns. So it's fairly realistic in terms of what
 %      you'd need to process if you were actually implementing a FleetByte
 %
 %    - Estimate the running direction (huh? but the rate gyro only returns
 %      the change in angle over the last second! we don't know the initial
 %      running direction right?) - well, you don't, but you can figure it
 %      out :) - that's part of the exercise.
 %      * REFERENCE: - given a direction vector, if you want to apply a
 %         rotation by a particular angle to this vector, you simply 
 %         multiply the vector by the corresponding rotation matrix:
 %
 %            d1=R*d;
 %
 %         Where d is the input direction vector (a unit-length, column
 %         vector with 2 components). R is the rotation matrix for 
 %         the amount of rotation you want:
 %
 %           R=[cos(theta) -sin(theta)
 %              sin(theta) cos(theta)];
 %
 %         'theta' is in radians. Finally, d1 is the resulting direction vector.
 %
 %    - Estimate the running speed in Km/h - This is *not* returned by any
 %      of the sensor readings, so you have to estimate it (carefully). 
 %
 %    Goal: To get the estimates as close as possible to the real value for
 %          the relevant quantities above. The last part of the script calls
 %          the imulation code to plot the real values against your estimares
 %          so you can see how well you're doing. In particular, you want the
 %          RMS of each measurement to be as close to 0 as possible.
 %          RMS is a common measure of error, and corresponds to the square
 %          root of the average squared error between a measurement and the
 %          corresponding estimate, taken over time. 
 %    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Replace with your computation of position, the map is 512x512 pixels in size
 theta = max(min(Rg, delta_t), -delta_t);
 % fprintf(2,'theta=%f\n', theta);

 % fprintf(2,'idx=%f\n', idx);

 if (idx == 1) 
    xyz = MPS;                % Replace with your computation of position, the map is 512x512 pixels in size
    di =[0 1];                % Replace with your computation for running direction, this should be a 2D unit vector
    vel = 5;                  % Replace with your computation of running velocity, in Km/h

 else
    %making the prediction after iteration 2 it computes and estimate the position in iteration 3

    if(idx == 2)
        % current x-y position is not preditable yet
        xyz = MPS;
        % fprintf(2,'xyz=[%f %f %f]\n',xyz(1),xyz(2),xyz(3));
    else
        xyz(1) = 0.6 * MPS(1) + 0.4 * (pred_xy(1) + prev_xyz(1));                % Replace with your computation of position, the map is 512x512 pixels in size
        xyz(2) = 0.6 * MPS(2) + 0.4 * (pred_xy(2) + prev_xyz(2));
        xyz(3) = MPS(3);
        % fprintf(2,'xyz=[%f %f %f]\n',xyz(1),xyz(2),xyz(3));
        % fprintf(2,'xyz_changes=[%f %f]\n',xyz(1)-prev_xyz(1),xyz(2)-prev_xyz(2));
    end;

    distance = sqrt((xyz(1) - prev_xyz(1))^2 + (xyz(2) - prev_xyz(2))^2);

    direction_vector_2D = [xyz(1)-prev_xyz(1)
                            xyz(2)-prev_xyz(2)];

    % Smoother
    Rg = 0.3 * prev_rg + 0.7 * Rg

    R=[cos(mean(hist_theta)) -sin(mean(hist_theta))
        sin(mean(hist_theta)) cos(mean(hist_theta))];

    % fprintf(2,'mean(hist_theta)=%f]\n',mean(hist_theta));

    pred_di = R * direction_vector_2D;
    % fprintf(2,'pred_di=[%f %f]\n',pred_di(1),pred_di(2));
    
    di_error = abs(atan(di(2)/di(1)) - atan(direction_vector_2D(2)/direction_vector_2D(1)));
    % fprintf(2,'di_error = %f\n', di_error);
    hist_di_error(end+1) = di_error;
    % fprintf(2,'hist_di_error(end)=%f\n', hist_di_error(end));

    if (idx > 10)
        % fprintf(2,'if (idx > 10)\n');
        
        sigma = std(hist_di_error);
        mean_error = mean(hist_di_error);
        % fprintf(2,'sigma = %f, mean_error = %f\n', std(hist_di_error), mean(hist_di_error));
        likelihood_DV2D_coef = (1 / (sqrt(2 * pi * sigma^2))) * exp(-(di_error - mean_error)^2 / (2 * sigma^2));
        % fprintf(2,'likelihood_DV2D_coef = %f\n', likelihood_DV2D_coef);

        di(1) = mean([0.7 (1 - likelihood_DV2D_coef)]) * di(1) + mean([0.3 (likelihood_DV2D_coef)]) * (0.4 * pred_di(1) + 0.6 * direction_vector_2D(1));
        di(2) = mean([0.7 (1 - likelihood_DV2D_coef)]) * di(2) + mean([0.3 (likelihood_DV2D_coef)]) * (0.4 * pred_di(2) + 0.6 * direction_vector_2D(2));

        % fprintf(2,'di=[%f %f]\n',di(1),di(2));
    else
        % fprintf(2,'if (idx <= 10)\n');
        di = direction_vector_2D;
    end;

    time_interval = 1;

    curr_vel = (distance / time_interval) * (3600 / 1000);
    curr_vel = max(min(15, curr_vel), 5);       % upper bound and lower bound
    
    last_five(mod(idx, 5) + 1) = curr_vel; % update the latest five velocity noisy data

    if idx >= 2 && idx < 3
        % fprintf(2,'if idx >= 2 && idx < 3\n');
        vel = curr_vel;
    else if idx >= 3 && idx <= 5  % linear approximation
        % fprintf(2,'else if idx >= 3 && idx <= 5\n');
        smoother_coef = abs(curr_vel - mean(hist_vel(2:end)));
        smoother_coef = min(10, smoother_coef) / 10;
        smoother_coef = 0.8 * smoother_coef;
        % fprintf(2,'smoother_coef=%f\n', smoother_coef);
        vel = (1 - smoother_coef) * curr_vel + smoother_coef * mean(hist_vel(2:end));
    else    % normalize distributed noise
        % fprintf(2,'else idx > 5\n');
        sigma = 1.5; % SD
        mean_val = mean(last_five);
        smoother_coef_curr_vel = (1 / (sqrt(2 * pi * sigma^2))) * exp(-(curr_vel - mean_val)^2 / (2 * sigma^2));
        smoother_coef_prev_vel = (1 / (sqrt(2 * pi * sigma^2))) * exp(-(prev_vel - mean_val)^2 / (2 * sigma^2));
        % fprintf(2,'smoother_coef_curr_vel=%f\n', smoother_coef_curr_vel);
        % fprintf(2,'smoother_coef_prev_vel=%f\n', smoother_coef_prev_vel);
        smoother_coef_sum = smoother_coef_curr_vel + smoother_coef_prev_vel;
        % fprintf(2,'smoother_coef_sum=%f\n', smoother_coef_sum);
        % smoother_coef = abs(curr_vel - prev_vel);
        % smoother_coef = min(10, smoother_coef) / 10;
        % smoother_coef = 0.8 * smoother_coef;
        % fprintf(2,'smoother_coef=%f\n', smoother_coef);
        % vel = mean([(smoother_coef_curr_vel / smoother_coef_sum) (1 - smoother_coef)]) * curr_vel + mean([(smoother_coef_prev_vel / smoother_coef_sum) smoother_coef]) * prev_vel;
        vel = smoother_coef_curr_vel * curr_vel + smoother_coef_prev_vel * prev_vel + (1 - smoother_coef_sum) * (0.5 * mean_val + 0.5 * mean(hist_vel(end-4:end)));
    end;
    end;

    % fprintf(2,'idx=%f\n', idx);

    % fprintf(2,'curr_vel=%f\n', curr_vel);
    % fprintf(2,'prev_vel=%f\n', prev_vel);
    % fprintf(2,'vel=%f\n', vel);

    pred_xy = [di(1) / norm(di) * vel * (1000/3600), di(2) / norm(di) * vel * (1000/3600)]
    % fprintf(2,'pred_xy=[%f %f]\n',pred_xy(1),pred_xy(2));

 end;


% Remove DC component
 HRS = HRS - mean(HRS);
 
 Fs = 3500;  % Sampling frequency
 L = length(HRS);  % Length of the signal

 % Continue with your FFT calculations
 Y = fft(HRS);
 P2 = abs(Y/L);
 P1 = P2(1:L/2+1);
 P1(2:end-1) = 2*P1(2:end-1);
 
 f = Fs*(0:(L/2))/L;
 
 %% Plot the amplitude spectrum
 %figure(6);
 %plot(f, P1);
 %title('Single-Sided Amplitude Spectrum of HRS Signal');
 %xlabel('Frequency (Hz)');
 %ylabel('|P1(f)|');
 
 % Find the peak in the spectrum
 [sorted_P1, sorted_indices] = sort(P1, 'descend');
 top_frequencies = f(sorted_indices(1:7));
 top_indices = sorted_indices(1:7);
 
 filtered_frequencies = [];

 for k = 1:length(top_indices)
    if abs(f(top_indices(k)) - prev_hr) < 25
        filtered_frequencies(end+1) = f(top_indices(k));
    end;
 end;

 % [~, i] = max(P1);  % Index of the peak frequency
 hr = max(filtered_frequencies);  % Dominant frequency in Hz
 
 % Print the index
 % fprintf(2, 'i=%d\n', i);  % Correctly prints idx as an integer

 if (debug==1)
     figure(5);clf;plot(HRS);
     fprintf(2,'****** For this frame idx = %f: *******\n', idx);
     fprintf(2,'MPS=[%f %f %f]\n',MPS(1),MPS(2),MPS(3));
     fprintf(2,'Rate gyro=%f\n',Rg);
     fprintf(2,'Distance traveled=%f\n',distance);
     fprintf(2,'direction_2D=[%f %f]\n',direction_vector_2D(1),direction_vector_2D(2));
     fprintf(2,'---> Press [ENTER] on the Matlab/Octave terminal to continue...\n');
     drawnow;
     pause;
 end;
 
 %%% SOLUTION:   

 prev_xyz = xyz;
 prev_vel = vel;
 prev_hr = hr;
 hist_theta(end + 1) = theta;
 hist_vel(end + 1) = vel;

 %%%%%%%%%%%%%%%%%%  DO NOT CHANGE ANY CODE BELOW THIS LINE %%%%%%%%%%%%%%%%%%%%%

 % Let's use the simulation script to plot your estimates against the real values
 % of the quantities of interest
 Sim1(map, xyz,hr,di,vel);
 idx=idx+1; 
end;

%%%%% Interesting links you may want to browse - I used these while designing this exercise.
% https://www.rohm.com/electronics-basics/sensor/pulse-sensor
% https://valencell.com/blog/optical-heart-rate-monitoring-what-you-need-to-know/
% https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html#product-details
