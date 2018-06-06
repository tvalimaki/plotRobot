function h = plotRobot(varargin)
% PLOTROBOT Plots the robot pose and covariance
%   PLOTROBOT(POSE) plots the robot pose POSE, where POSE is either [x,y]
%   or [x,y,theta].
%   PLOTROBOT(POSE, COV) additionally plots the error ellipse with 50%
%   confidence interval using the covariance matrix COV.
%   PLOTROBOT(POSE, COV, CONF) plots the error ellipse with confidence
%   interval CONF.
%   PLOTROBOT(__, NAME, VALUE) additionally specify properties using
%   one or more NAME, VALUE pairs. See list of properties below.
%   PLOTROBOT(AX, __) creates the plot in the axes specified by AX instead
%   of the current axes.
%   H = PLOTROBOT(__) returns the handles to the graphic objects.
%
%   The function plots a circular marker with a 'nose', denoting the
%   heading and x,y coordinates of a robot. The length of the 'nose' is
%   automatically calculated to fit the size of the circular marker. A
%   callback function is added to the parent figure properties to keep
%   resizing the marker for all previous robots drawn with PLOTROBOT on the
%   same axes, given that PLOTROBOT was not used to plot on another axes
%   inbetween.
%
%   Properties:
%
%   Color      - RGB triplet or one of the common color names. Default 'k'.
%   Alpha      - FaceAlpha to use for error ellipse. Scalar in range [0,1],
%                default 0.2.
%   MarkerSize - Marker size. Positive value, default 15.
%   LineWidth  - Line width. Positive value, default 2.
%   LineStyle  - Line style to use for error ellipse. One of
%                'none' (default) | '-' | '--' | ':' | '-.'
%
% Copyright (c) 2017-2018 Tuomas Välimäki

% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files
% (the "Software"), to deal in the Software without restriction, including
% without limitation the rights to use, copy, modify, merge, publish,
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the
% following conditions:
%
% The above copyright notice and this permission notice shall be included
% in all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
% IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
% CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
% OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
% THE USE OR OTHER DEALINGS IN THE SOFTWARE.

    persistent ax markerArray el

    % Check whether axes was given as first input and clear markers if not
    % the same axes as before
    if isa(varargin{1}, 'matlab.graphics.axis.Axes')
        if ax~=varargin{1}
            markerArray = {};
            delete(el);
        end
        ax = varargin{1};
        varargin = varargin{2:end};
    else
        if ax~=gca
            markerArray = {};
            delete(el);
        end
        ax = gca;
    end

    % Set defaults for the parameters
    defaultConfidence = 0.5;
    defaultColor = 'k';
    defaultAlpha = 0.2;
    defaultMarkerSize = 15;
    defaultLineWidth = 2;
    defaultLineStyle = 'none';

    % Create input parser and parse inputs
    p = inputParser;
    p.FunctionName = 'plotRobot';
    validPose = @(x) isnumeric(x) && isreal(x) && ...
                     2<=numel(x) && numel(x)<=3;
    validCov = @(x) isnumeric(x) && isreal(x) && ismatrix(x) && ...
                    ~diff(size(x)) && issymmetric(x) && ...
                    all(eig((x+x')/2)>=0);
    validConfidence = @(x) isnumeric(x) && isreal(x) && isscalar(x) && ...
                           0<=x && x<=1;
    addRequired(p, 'pose', validPose);
    addOptional(p, 'covariance', {}, validCov);
    addOptional(p, 'confidence', defaultConfidence, validConfidence);
    addParameter(p, 'color', defaultColor);
    addParameter(p, 'alpha', defaultAlpha);
    addParameter(p, 'markerSize', defaultMarkerSize);
    addParameter(p, 'lineWidth', defaultLineWidth);
    addParameter(p, 'lineStyle', defaultLineStyle);

    parse(p, varargin{:});
    pose = p.Results.pose;
    covariance = p.Results.covariance;
    confidence = p.Results.confidence;
    color = p.Results.color;
    alpha = p.Results.alpha;
    markerSize = p.Results.markerSize;
    lineWidth = p.Results.lineWidth;
    lineStyle = p.Results.lineStyle;

    % Plot robot position and auto-resize the axis to accommodate the data
    set(ax, 'XLimMode', 'auto', 'YLimMode', 'auto');
    handles = plot(ax, pose(1), pose(2), 'o', 'Color', color, ...
                   'MarkerSize', markerSize, 'LineWidth', lineWidth);

    % Get hold state and clear markers if there's no hold even if the axes
    % is the same as before
    wasHold = ishold(ax);
    if ~wasHold
        markerArray = {};
        delete(el);
    end
    hold(ax, 'on');

    % Plot robot orientation if given
    if numel(pose)>2
        handle = plot(ax, pose(1), pose(2), ...
                      'Color', color, 'LineWidth', lineWidth);
        % Don't clip the 'nose' even if it extends outside the axes
        set(handle, 'Clipping', 'off');
        % Add the new 'nose' marker to the array to be resized
        markerArray{end+1} = @(x) resizeMarker(handle, pose, markerSize);
        % Resize all the markers to the current axes
        resizeMarkerArray(markerArray);
        % Set callback to figure resizing and axis limit changes
        set(ax.Parent, 'SizeChangedFcn', ...
            @(obj,arg) resizeMarkerArray(markerArray));
        el = listener(ax, {'XLim', 'YLim'}, 'PostSet', ...
                      @(obj,arg) resizeMarkerArray(markerArray));
        handles = vertcat(handles, handle);
    end

    % Plot covariance if given
    if ~isempty(covariance)
        assert(numel(pose)==size(covariance, 1), ...
               'Dimension mismatch between pose and covariance.');

        % Compute quantile for the desired percentile
        k = sqrt(chi2inv(confidence, numel(pose)));

        n = 100; % Number of points around ellipse
        p = 0:pi/n:2*pi; % angles around a circle

        [eigvec, eigval] = eig(covariance(1:2, 1:2));
        xy = [cos(p'), sin(p')] * sqrt(eigval) * eigvec';
        x = xy(:,1);
        y = xy(:,2);

        handle = fill(ax, pose(1)+k*x, pose(2)+k*y, color, ...
                      'EdgeColor', color, 'LineStyle', lineStyle, ...
                      'LineWidth', lineWidth,'FaceAlpha', alpha);

        handles = vertcat(handles, handle);
    end

    % Restore hold state
    if ~wasHold
        hold(ax, 'off');
    end

    % Return the handles if asked
    if nargout>0
        h = handles;
    end

    function resizeMarker(h, pose, markerSize)
    % RESIZEMARKER Resizes the robot 'nose' markers to the correct marker
    % size given the current axes limits
        ax_ = h.Parent; % Get axes
        lim = axis(ax_); % Get axes limits
        currentUnits = ax_.Units; % Save current units
        ax_.Position;
        ax_.Units = 'Points'; % Change units to points
        axpos = ax_.Position; % Get axes width and height
        ax_.Units = currentUnits; % Restore units
        % Calculate scale from points to data units
        scale = diff(reshape(lim, [2,2]))./axpos(3:4);
        a = markerSize.*scale;
        % Redraw the 'nose'
        set(h, 'XData', [pose(1), pose(1)+a(1)*cos(pose(3))], ...
            'YData', [pose(2), pose(2)+a(2)*sin(pose(3))]);
        axis(ax_, lim); % Restore limits
    end

    function resizeMarkerArray(markerArray)
    % RESIZEMARKERARRAY Calls all resizing functions in the array
        for i=1:length(markerArray)
            markerArray{i}();
        end
    end
end
