function model = spinn3d_train_v2(datasetMat, outMat)
% Train feedforward NN to predict [alpha1..alpha4, g] from features.
% datasetMat: path to spinn3d_v2_dataset.mat
% outMat: where to save the trained model (default 'spinn3d_v2_net.mat')

if nargin < 1 || isempty(datasetMat), datasetMat = 'spinn3d_v2_dataset.mat'; end
if nargin < 2 || isempty(outMat),     outMat = 'spinn3d_v2_net.mat'; end

S = load(datasetMat);
X = S.X; Y = S.Y;
[M, D] = size(X);

% split
rng(0);
idx = randperm(M);
nval = max(1000, round(0.1*M));
ival = idx(1:nval);
itr  = idx(nval+1:end);

Xtr = X(itr,:); Ytr = Y(itr,:);
Xva = X(ival,:);Yva = Y(ival,:);

layers = [
    featureInputLayer(D,"Normalization","zscore","Name","in")
    fullyConnectedLayer(128,"Name","fc1")
    reluLayer("Name","relu1")
    fullyConnectedLayer(128,"Name","fc2")
    reluLayer("Name","relu2")
    fullyConnectedLayer(5,"Name","fc_out")
    regressionLayer("Name","reg")];

opts = trainingOptions("adam", ...
    "MiniBatchSize", 1024, ...
    "MaxEpochs", 35, ...
    "InitialLearnRate", 2e-3, ...
    "Shuffle","every-epoch", ...
    "ValidationData",{Xva,Yva}, ...
    "ValidationFrequency", max(1,floor(numel(itr)/1024/5)), ...
    "Plots","training-progress", ...
    "Verbose",false);

net = trainNetwork(Xtr, Ytr, layers, opts);

% simple metrics
Yhat = predict(net, Xva);
mae  = mean(abs(Yhat - Yva), 1);
fprintf('Validation MAE (alpha1..alpha4,g): '); fprintf('%.4f ', mae); fprintf('\n');

model = struct("net",net, "caps",S.caps, "params", S.params);
save(outMat,'-struct','model');
fprintf('Saved trained model to %s\n', outMat);
end
