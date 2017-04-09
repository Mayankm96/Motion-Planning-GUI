function potential(S, G, Obstacles)
%Draws a path from Start S to Goal G using potential field method

%%Defining the parameters for the problem
%Defining Region of Infuence of Obstacles
d_roi = 50*ones(1, length(Obstacles));

%Defining parameters
kr = 10^11*ones(1, length(Obstacles));
gamma = 2;
ka = 100;
rho = 200;
thresh = 10^(1);
step_size = 10^(-4);

%%Iterations begin
X = S;

while true
    
    U = 0; 
    Grad = [0; 0];

    for i = 1:length(Obstacles)
        [d, pt] = pointToPolyDist(X, Obstacles{i}.Vertices);
        % Repulsive Potential Field
        if d < d_roi(i)
            direction = (X-pt)/norm(X-pt, 2);
            %plot([X(1), pt(1)],[X(2), pt(2)],'--y')
            U = U - kr(i)/gamma*(1/d-1/d_roi(i))^gamma;
            Grad = Grad - (kr(i)*(1/d_roi(i)-1/d)/d^2)^(gamma-1)*direction;
        end
    end

       

    % Attractive Potential Field
    d_goal = norm(X-G, 2);
    if d_goal <=rho
        U = U + ka/2*d_goal^2;
        Grad = Grad - ka*(X-G); 
    else
        U = U + ka*rho*d_goal - 1/2*ka*d_goal^2;
        Grad = Grad - rho*ka*(X-G)/d_goal;
    end
    
    if norm(Grad)<thresh
       break;
    else
        X = X + step_size*Grad;
    end
    plot(X(1), X(2),'ob','MarkerSize',2, 'MarkerEdgeColor','b');
    disp(X);
    pause(0.005);

    if X(1)<0 || X(1)>1000 || X(2)<0 || X(2)>1000
        error('Outside the box!')
    end
end 

scatter(S(1),S(2), 'g', 'filled');
scatter(G(1),G(2), 'r', 'filled');

end

