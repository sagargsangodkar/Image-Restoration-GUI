matrix = [1 2 3;2 3 4;4 5 6];
[M,N,Z] = size(matrix);

for k = 1:M
    for l = 1:N
        F(k,l) = 0;
        for m = 1:M
            for n = 1:N
                F(k,l) = F(k,l) + matrix(m,n) * exp(-j*2*pi*l*n/N);
            end
            