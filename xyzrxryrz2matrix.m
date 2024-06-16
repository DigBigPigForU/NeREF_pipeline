% need to prepare the data.mat x y z rx ry rz
% return a matrix res in T[1]...T[16] 
% data = [];
T_t2c = [
0 0.99955 0.029996  -0.03459;
0  -0.03 0.99955 0.065924;
1 0 0 0.12;
0 0 0 1
];
[m,n] = size(data);
for i = 1:m
    R = rotationVectorToMatrix(data(i,4:6))';
    T_b2t(4,1:4) = [0 0 0 1];
    T_b2t(1:3,1:3) = R;
    T_b2t(1:3,4) = [data(i,1);data(i,2);data(i,3)]./1000;
    
    T_b2c = T_b2t*T_t2c;
    
    res_b2t(i,1:4) = T_b2t(1,1:4);
    res_b2t(i,5:8) = T_b2t(2,1:4);
    res_b2t(i,9:12) = T_b2t(3,1:4);
    res_b2t(i,13:16) = T_b2t(4,1:4);
    
    res_b2c(i,1:4) = T_b2c(1,1:4);
    res_b2c(i,5:8) = T_b2c(2,1:4);
    res_b2c(i,9:12) = T_b2c(3,1:4);
    res_b2c(i,13:16) = T_b2c(4,1:4);
end