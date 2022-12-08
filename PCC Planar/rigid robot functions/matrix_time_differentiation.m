function output = matrix_time_differentiation(M, x, x_dot)
    
    temp = zeros(size(M));

    for i = 1:1:length(x)
        temp = temp + diff(M,x(i)).*x_dot(i);
    end

output = temp;

end