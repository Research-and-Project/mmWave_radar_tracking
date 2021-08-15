%% Using Kalman Filter for Object Tracking
% This example shows how to use the |vision.KalmanFilter| object and
% |configureKalmanFilter| function to track objects.

% Copyright 2012 The MathWorks, Inc.

%%
% This example is a function with its main body at the top and helper
% routines in the form of 
% <matlab:helpview(fullfile(docroot,'toolbox','matlab','matlab_prog','matlab_prog.map'),'nested_functions')
% nested functions> below.
function kalmanFilterForTracking

%% Introduction
% The Kalman filter has many uses, including applications in control,
% navigation, computer vision, and time series econometrics. This example
% illustrates how to use the Kalman filter for tracking objects and focuses
% on three important features:

%%
% * Prediction of object's future location
% * Reduction of noise introduced by inaccurate detections
% * Facilitating the process of association of multiple objects to their
%   tracks

%% Challenges of Object Tracking
% Before showing the use of Kalman filter, let us first examine the
% challenges of tracking an object in a video. The following video shows a
% green ball moving from left to right on the floor.
showDetections();

%%
% The white region over the ball highlights the pixels detected using
% |vision.ForegroundDetector|, which separates moving objects from the
% background. The background subtraction only finds a portion of the ball
% because of the low contrast between the ball and the floor. In other
% words, the detection process is not ideal and introduces noise.
%
% To easily visualize the entire object trajectory, we overlay all video
% frames onto a single image. The "+" marks indicate the centroids computed
% using blob analysis.
showTrajectory();

%%
% Two issues can be observed:

%%
% # The region's center is usually different from the ball's center. In
%   other words, there is an error in the measurement of the ball's
%   location.
% # The location of the ball is not available when it is occluded by the
%   box, i.e. the measurement is missing.

%%
% Both of these challenges can be addressed by using the Kalman filter.

%% Track a Single Object Using Kalman Filter
% Using the video which was seen earlier, the |trackSingleObject| function
% shows you how to:

%% 
% * Create |vision.KalmanFilter| by using |configureKalmanFilter|
% * Use |predict| and |correct| methods in a sequence to eliminate noise
%   present in the tracking system
% * Use |predict| method by itself to estimate ball's location when
%   it is occluded by the box
%
% The selection of the Kalman filter parameters can be challenging. The
% |configureKalmanFilter| function helps simplify this problem. More
% details about this can be found further in the example.

%%
% The |trackSingleObject| function includes nested helper functions. The
% following top-level variables are used to transfer the data between the
% nested functions.  
frame            = [];  % A video frame
detectedLocation = [];  % The detected location
trackedLocation  = [];  % The tracked location
label            = '';  % Label for the ball
utilities        = [];  % Utilities used to process the video

%%
% The procedure for tracking a single object is shown below.
function trackSingleObject(param)
  % Create utilities used for reading video, detecting moving objects,
  % and displaying the results.
  utilities = createUtilities(param);

  isTrackInitialized = false;
  while ~isDone(utilities.videoReader)
    frame = readFrame();

    % Detect the ball.
    [detectedLocation, isObjectDetected] = detectObject(frame);

    if ~isTrackInitialized
      if isObjectDetected
        % Initialize a track by creating a Kalman filter when the ball is
        % detected for the first time.
        initialLocation = computeInitialLocation(param, detectedLocation);
        kalmanFilter = configureKalmanFilter(param.motionModel, ...
          initialLocation, param.initialEstimateError, ...
          param.motionNoise, param.measurementNoise);

        isTrackInitialized = true;
        trackedLocation = correct(kalmanFilter, detectedLocation);
        label = 'Initial';
      else
        trackedLocation = [];
        label = '';
      end

    else
      % Use the Kalman filter to track the ball.
      if isObjectDetected % The ball was detected.
        % Reduce the measurement noise by calling predict followed by
        % correct.
        predict(kalmanFilter);
        trackedLocation = correct(kalmanFilter, detectedLocation);
        label = 'Corrected';
      else % The ball was missing.
        % Predict the ball's location.
        trackedLocation = predict(kalmanFilter);
        label = 'Predicted';
      end
    end

    annotateTrackedObject();
  end % while
  
  showTrajectory();
end

%% 
% There are two distinct scenarios that the Kalman filter addresses:

%%
% * When the ball is detected, the Kalman filter first predicts its state
%   at the current video frame, and then uses the newly detected object
%   location to correct its state. This produces a filtered location.
% * When the ball is missing, the Kalman filter solely relies on its
%   previous state to predict the ball's current location.

%%
% You can see the ball's trajectory by overlaying all video frames.
param = getDefaultParameters();  % get Kalman configuration that works well
                                 % for this example
                                 
trackSingleObject(param);  % visualize the results

%% Explore Kalman Filter Configuration Options
% Configuring the Kalman filter can be very challenging. Besides basic
% understanding of the Kalman filter, it often requires experimentation in
% order to come up with a set of suitable configuration parameters. The
% |trackSingleObject| function, defined above, helps you to explore the
% various configuration options offered by the |configureKalmanFilter|
% function.
%
% The |configureKalmanFilter| function returns a Kalman filter object. You
% must provide five input arguments.
%
%   kalmanFilter = configureKalmanFilter(MotionModel, InitialLocation,
%            InitialEstimateError, MotionNoise, MeasurementNoise)

%%
% The *MotionModel* setting must correspond to the physical characteristics
% of the object's motion. You can set it to either a constant velocity or
% constant acceleration model. The following example illustrates the
% consequences of making a sub-optimal choice.
param = getDefaultParameters();         % get parameters that work well
param.motionModel = 'ConstantVelocity'; % switch from ConstantAcceleration
                                        % to ConstantVelocity
% After switching motion models, drop noise specification entries
% corresponding to acceleration.
param.initialEstimateError = param.initialEstimateError(1:2);
param.motionNoise          = param.motionNoise(1:2);

trackSingleObject(param); % visualize the results

%%
% Notice that the ball emerged in a spot that is quite different from the
% predicted location. From the time when the ball was released, it was
% subject to constant deceleration due to resistance from the carpet.
% Therefore, constant acceleration model was a better choice. If you kept
% the constant velocity model, the tracking results would be sub-optimal no
% matter what you selected for the other values.

%%
% Typically, you would set the *InitialLocation* input to the location
% where the object was first detected. You would also set the
% *InitialEstimateError* vector to large values since the initial state may
% be very noisy given that it is derived from a single detection. The
% following figure demonstrates the effect of misconfiguring these
% parameters.

param = getDefaultParameters();  % get parameters that work well
param.initialLocation = [0, 0];  % location that's not based on an actual detection 
param.initialEstimateError = 100*ones(1,3); % use relatively small values

trackSingleObject(param); % visualize the results

%%
% With the misconfigured parameters, it took a few steps before the
% locations returned by the Kalman filter align with the actual trajectory
% of the object.

%%
% The values for *MeasurementNoise* should be selected based on the
% detector's accuracy. Set the measurement noise to larger values for a
% less accurate detector. The following example illustrates the noisy
% detections of a misconfigured segmentation threshold. Increasing the
% measurement noise causes the Kalman filter to rely more on its internal
% state rather than the incoming measurements, and thus compensates for the
% detection noise.

param = getDefaultParameters();
param.segmentationThreshold = 0.0005; % smaller value resulting in noisy detections
param.measurementNoise      = 12500;  % increase the value to compensate 
                                      % for the increase in measurement noise

trackSingleObject(param); % visualize the results

%%
% Typically objects do not move with constant acceleration or constant
% velocity. You use the *MotionNoise* to specify the amount of deviation
% from the ideal motion model. When you increase the motion noise, the
% Kalman filter relies more heavily on the incoming measurements than on
% its internal state. Try experimenting with *MotionNoise* parameter to
% learn more about its effects.

%%
% Now that you are familiar with how to use the Kalman filter and how to
% configure it, the next section will help you learn how it can be used for
% multiple object tracking.

%%
% *Note:* In order to simplify the configuration process in the above
% examples, we used the |configureKalmanFilter| function. This function
% makes several assumptions. See the function's documentation for details.
% If you require greater level of control over the configuration process,
% you can use the |vision.KalmanFilter| object directly.

%% Track Multiple Objects Using Kalman Filter
%
% Tracking multiple objects poses several additional challenges:

%% 
% * Multiple detections must be associated with the correct tracks
% * You must handle new objects appearing in a scene 
% * Object identity must be maintained when multiple objects merge into a
%   single detection
%
% The |vision.KalmanFilter| object together with the
% |assignDetectionsToTracks| function can help to solve the problems of

%%
% * Assigning detections to tracks
% * Determining whether or not a detection corresponds to a new object, 
%   in other words, track creation
% * Just as in the case of an occluded single object, prediction can be
%   used to help separate objects that are close to each other
%
% To learn more about using Kalman filter to track multiple objects, see
% the example titled <matlab:showdemo('MotionBasedMultiObjectTrackingExample')
% Motion-Based Multiple Object Tracking>.

%% Utility Functions Used in the Example
% Utility functions were used for detecting the objects and displaying the
% results. This section illustrates how the example implemented these
% functions.

%%
% Get default parameters for creating Kalman filter and for segmenting the
% ball.
function param = getDefaultParameters
  param.motionModel           = 'ConstantAcceleration';
  param.initialLocation       = 'Same as first detection';
  param.initialEstimateError  = 1E5 * ones(1, 3);
  param.motionNoise           = [25, 10, 1];
  param.measurementNoise      = 25;
  param.segmentationThreshold = 0.05;
end

%%
% Read the next video frame from the video file.
function frame = readFrame()
  frame = step(utilities.videoReader);
end

%%
% Detect and annotate the ball in the video.
function showDetections()
  param = getDefaultParameters();
  utilities = createUtilities(param);
  trackedLocation = [];

  idx = 0;
  while ~isDone(utilities.videoReader)
    frame = readFrame();
    detectedLocation = detectObject(frame);
    % Show the detection result for the current video frame.
    annotateTrackedObject();

    % To highlight the effects of the measurement noise, show the detection
    % results for the 40th frame in a separate figure.
    idx = idx + 1;
    if idx == 40
      combinedImage = max(repmat(utilities.foregroundMask, [1,1,3]), frame);
      figure, imshow(combinedImage);
    end
  end % while
  
  % Close the window which was used to show individual video frame.
  uiscopes.close('All'); 
end

%%
% Detect the ball in the current video frame.
function [detection, isObjectDetected] = detectObject(frame)
  grayImage = rgb2gray(frame);
  utilities.foregroundMask = step(utilities.foregroundDetector, grayImage);
%   imshow(utilities.foregroundMask) % zzh
  detection = step(utilities.blobAnalyzer, utilities.foregroundMask);
  if isempty(detection)
    isObjectDetected = false;
  else
    % To simplify the tracking process, only use the first detected object.
    detection = detection(1, :);
    isObjectDetected = true;
  end
end

%%
% Show the current detection and tracking results.
function annotateTrackedObject()
  accumulateResults();
  % Combine the foreground mask with the current video frame in order to
  % show the detection result.
  combinedImage = max(repmat(utilities.foregroundMask, [1,1,3]), frame);

  if ~isempty(trackedLocation)
    shape = 'circle';
    region = trackedLocation;
    region(:, 3) = 5;
    combinedImage = insertObjectAnnotation(combinedImage, shape, ...
      region, {label}, 'Color', 'red');
  end
  step(utilities.videoPlayer, combinedImage);
end

%%
% Show trajectory of the ball by overlaying all video frames on top of 
% each other.
function showTrajectory
  % Close the window which was used to show individual video frame.
  uiscopes.close('All'); 
  
  % Create a figure to show the processing results for all video frames.
  figure; imshow(utilities.accumulatedImage/2+0.5); hold on;
  plot(utilities.accumulatedDetections(:,1), ...
    utilities.accumulatedDetections(:,2), 'k+');
  
  if ~isempty(utilities.accumulatedTrackings)
    plot(utilities.accumulatedTrackings(:,1), ...
      utilities.accumulatedTrackings(:,2), 'r-o');
    legend('Detection', 'Tracking');
  end
end

%%
% Accumulate video frames, detected locations, and tracked locations to
% show the trajectory of the ball.
function accumulateResults()
  utilities.accumulatedImage      = max(utilities.accumulatedImage, frame);
  utilities.accumulatedDetections ...
    = [utilities.accumulatedDetections; detectedLocation];
  utilities.accumulatedTrackings  ...
    = [utilities.accumulatedTrackings; trackedLocation];
end

%%
% For illustration purposes, select the initial location used by the Kalman
% filter.
function loc = computeInitialLocation(param, detectedLocation)
  if strcmp(param.initialLocation, 'Same as first detection')
    loc = detectedLocation;
  else
    loc = param.initialLocation;
  end
end

%%
% Create utilities for reading video, detecting moving objects, and
% displaying the results.
function utilities = createUtilities(param)
  % Create System objects for reading video, displaying video, extracting
  % foreground, and analyzing connected components.
  utilities.videoReader = vision.VideoFileReader('singleball.mp4');
  utilities.videoPlayer = vision.VideoPlayer('Position', [100,100,500,400]);
  utilities.foregroundDetector = vision.ForegroundDetector(...
    'NumTrainingFrames', 10, 'InitialVariance', param.segmentationThreshold);
  utilities.blobAnalyzer = vision.BlobAnalysis('AreaOutputPort', false, ...
    'MinimumBlobArea', 70, 'CentroidOutputPort', true);

  utilities.accumulatedImage      = 0;
  utilities.accumulatedDetections = zeros(0, 2);
  utilities.accumulatedTrackings  = zeros(0, 2);
end



end
