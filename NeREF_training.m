% Check the size of variables
disp(size(x))
disp(size(y))

% Split data into training and validation sets
numTrain = floor(0.8 * size(x, 1));
xTrain = x(1:numTrain, :);
yTrain = y(1:numTrain, :);
xVal = x(numTrain+1:end, :);
yVal = y(numTrain+1:end, :);

% Define the network architecture
layers = [
    imageInputLayer([size(x, 2) 1 1])  % Input layer
    fullyConnectedLayer(128)           % First hidden layer
    reluLayer                          % Activation function
    fullyConnectedLayer(128)           % Second hidden layer
    reluLayer                          % Activation function
    fullyConnectedLayer(128)           % Third hidden layer
    reluLayer                          % Activation function
    fullyConnectedLayer(128)           % Fourth hidden layer
    reluLayer                          % Activation function
    fullyConnectedLayer(128)           % Fifth hidden layer
    reluLayer                          % Activation function
    fullyConnectedLayer(128)           % Sixth hidden layer
    reluLayer                          % Activation function
    fullyConnectedLayer(128)           % Seventh hidden layer
    reluLayer                          % Activation function
    fullyConnectedLayer(128)           % Eighth hidden layer
    reluLayer                          % Activation function
    fullyConnectedLayer(size(y, 2))    % Output layer
    regressionLayer                    % Regression layer
];

% Training options
options = trainingOptions('adam', ... % Use Adam optimizer
    'MaxEpochs', 50, ...              % Maximum number of epochs
    'MiniBatchSize', 64, ...          % Mini-batch size
    'InitialLearnRate', 0.001, ...    % Initial learning rate
    'ValidationData', {reshape(xVal', [size(x, 2), 1, 1, size(xVal, 1)]), yVal}, ... % Validation data
    'Plots', 'training-progress', ... % Plot training progress
    'Verbose', false);                % Do not display verbose information

% Train the network
NeREF = trainNetwork(reshape(xTrain', [size(x, 2), 1, 1, size(xTrain, 1)]), yTrain, layers, options);

% Save the network
save('NeREF.mat', 'NeREF');
