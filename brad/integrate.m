function integral = integrate(vec, delta_t)
integral = zeros(size(vec));
integral(1) = vec(1)*delta_t;
for i = 2:size(vec)
    integral(i) = integral(i-1) + vec(i)*delta_t;
end
end