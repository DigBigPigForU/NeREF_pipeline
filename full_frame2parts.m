clear all
clc

% 1. 指定图片文件夹路径
imageFolderPath = 'D:\github_NeREF'; % 替换为包含PNG图片的文件夹路径

% 2. 获取图片文件列表
imageFiles = dir(fullfile(imageFolderPath, '*.tif'));

% 创建保存拆分部分的文件夹
numParts = 25; % 分成25个部分
partFolderPrefix = 'part'; % 文件夹前缀
outputFolder = fullfile(imageFolderPath, 'splitted_tif_frames');
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

for i = 1:numParts
    partFolder = fullfile(outputFolder, [partFolderPrefix, num2str(i)]);
    if ~exist(partFolder, 'dir')
        mkdir(partFolder);
    end
end

% 循环处理每个图片文件
for fileIdx = 1:length(imageFiles)
    imageFile = fullfile(imageFolderPath, imageFiles(fileIdx).name);
    
    % 读取图像
    frame = imread(imageFile);
    
    % 将像素值转换为uint8类型，确保像素值在0到255之间
    frame = uint8(frame);
    
    % 删除最后三行和最后三列
    frame = frame(1:end-3, 1:end-3, :);
    
    % 拆分图像
    bands_order = [21 22 23 24 25; 16 17 18 19 20; 11 12 13 14 15; 6 7 8 9 10; 1 2 3 4 5];
    DN_white_in_bands = zeros(ceil(size(frame, 1) / 5), ceil(size(frame, 2) / 5), numParts);

    for row = 1:size(frame, 1)
        for col = 1:size(frame, 2)
            r = mod(row, 5);
            c = mod(col, 5);
            if r == 0
                r = 5;
            end
            if c == 0
                c = 5;
            end
            DN_white_in_bands((row - r) / 5 + 1, (col - c) / 5 + 1, bands_order(r, c)) = frame(row, col);
        end
    end

    % 保存图像部分
    for i = 1:numParts
        currentPart = DN_white_in_bands(:, :, i);
        
        % 保存原始部分
        partOutputFilename = fullfile(outputFolder, [partFolderPrefix, num2str(i)], ...
            sprintf('%s_part%d.png', imageFiles(fileIdx).name(1:end-4), i));
        currentPart = uint8(currentPart); % 将像素值转换为uint8类型，确保像素值在0到255之间
        imwrite(currentPart, partOutputFilename, 'png');
    end

    disp(['图片 ', imageFiles(fileIdx).name, ' 的拆分处理完成！']);
end
