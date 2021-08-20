function plotStateSpace(i, X_log,X_pre)
plot(X_log(1,1:i-1),X_log(2,1:i-1),'o',X_pre(1,:,i),X_pre(2,:,i),'ro')
hold on
plot(X_log(1,i),X_log(2,i),'ko','MarkerFaceColor','k');
plot(0,0,'x')
hold off
grid on
title(['Iteration ',num2str(i)])
xlabel('$x_1$','interpreter','latex')
ylabel('$x_2$','interpreter','latex')
axis([-0.5,11.5,-5.5,11.5])
end

