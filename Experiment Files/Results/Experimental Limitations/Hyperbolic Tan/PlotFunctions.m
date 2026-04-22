x = linspace(-4, 4, 1000);

% Plotting the hyperbolic tangent and sign functions
figure;
plot(x, sign(x), 'Color', [0, 0, 0.7], 'LineWidth', 1.5); hold on;
plot(x, tanh(10*x), 'Color', [0.7, 0, 0], 'LineWidth', 1.5);
legend('sign($x$)', 'tanh($10x$)', 'Interpreter', 'latex');
xlabel('$x$', 'Interpreter', 'latex');
ylabel('$y$', 'Interpreter', 'latex');
grid on; box on;