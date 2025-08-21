# REAL--TIME-OBJECT-TRACKING-USING-IMAGING-TECHNIQUES
Developed a MATLAB-based system to track moving objects using Background Subtraction and KLT algorithm. Implemented preprocessing, feature extraction, and trajectory visualization to achieve accurate tracking under varying conditions like occlusion and background clutter.
% Load video
videoFile = "C:\Users\KIIT\Downloads\video 2.mp4";
videoReader = VideoReader(videoFile);

% Read first frame as background
background = readFrame(videoReader);
background = rgb2gray(background);

% Smooth background
h = fspecial("average", [3 3]);
backgroundFiltered = imfilter(background, h);

% Number of frames to process
numFrames = min(450, floor(videoReader.Duration * videoReader.FrameRate));

% Initialize centroid storage
missileCentroids = [];

figure;

for k = 1:numFrames
    if hasFrame(videoReader)
        % Read current frame and preprocess
        currentFrame = readFrame(videoReader);
        grayFrame = rgb2gray(currentFrame);
        grayFiltered = imfilter(grayFrame, h);

        % Background subtraction
        Id = imabsdiff(grayFiltered, backgroundFiltered);

        % Thresholding
        threshold = 25;
        bw = Id > threshold;

        % Morphological cleanup
        bw = bwareaopen(bw, 100);
        bw = imclose(bw, strel('rectangle', [5, 5]));

        % Region properties
        stats = regionprops(bw, 'BoundingBox', 'Area', 'Centroid', 'Eccentricity');

        % Display setup
        subplot(1, 3, 1); imshow(background); title("Background");
        subplot(1, 3, 2); imshow(currentFrame); title(['Current Frame#' num2str(k)]); hold on;
        subplot(1, 3, 3); imshow(bw); title("Foreground (Motion)");

        % Look for elongated missile-like object
        for i = 1:length(stats)
            area = stats(i).Area;
            ecc = stats(i).Eccentricity;

            if area > 400 && ecc > 0.85
                bb = stats(i).BoundingBox;
                centroid = stats(i).Centroid;

                % Draw results
                subplot(1, 3, 2);
                rectangle('Position', bb, 'EdgeColor', 'r', 'LineWidth', 2);
                plot(centroid(1), centroid(2), 'g*', 'MarkerSize', 8);
                text(centroid(1)+5, centroid(2), ...
                    sprintf('(%0.0f,%0.0f)', centroid(1), centroid(2)), ...
                    'Color', 'yellow', 'FontSize', 8);

                % Store [frame, x, y]
                missileCentroids = [missileCentroids; k, centroid];
                break;  % Only one object per frame
            end
        end
        hold off;
        drawnow;
    end
end

% ---- Plot Full Trajectory ----
figure('Name', 'Missile Trajectory');
imshow(currentFrame); hold on;
title('Missile Centroid Path');

% Extract and plot coordinates
x = missileCentroids(:, 2);
y = missileCentroids(:, 3);
plot(x, y, 'r-', 'LineWidth', 2);
plot(x, y, 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');

% Label start and end
text(x(1), y(1), 'Start', 'Color', 'cyan', 'FontSize', 10, 'FontWeight', 'bold');
text(x(end), y(end), 'End', 'Color', 'magenta', 'FontSize', 10, 'FontWeight', 'bold');
hold off;

% ---- Extract 10 evenly spaced centroid points ----
totalPoints = size(missileCentroids, 1);
if totalPoints >= 10
    idx = round(linspace(1, totalPoints, 10));
    selected10 = missileCentroids(idx, :);
else
    warning('Less than 10 centroids detected. Returning available ones.');
    selected10 = missileCentroids;
end

% ---- Save to table and CSV ----
centroidTable = table(selected10(:,1), selected10(:,2), selected10(:,3), ...
    'VariableNames', {'Frame', 'X_Centroid', 'Y_Centroid'});
disp(centroidTable);

writetable(centroidTable, 'Missile_10_Centroids.csv');
