function [fitresult, gof] = createFit(X1ss, Yajuste)
%CREATEFIT(X1SS,YAJUSTE)
%  Create a fit.
%
%  Data for 'Ajuste Exp 8' fit:
%      X Input : X1ss
%      Y Output: Yajuste
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 28-Jan-2019 22:49:01


%% Fit: 'Ajuste Exp 8'.
[xData, yData] = prepareCurveData( X1ss, Yajuste );

% Set up fittype and options.
ft = fittype( 'poly1' );

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'Ajuste Exp 8' );
h = plot( fitresult, xData, yData );
legend( h, 'Yajuste vs. X1ss', 'Ajuste Exp 8', 'Location', 'NorthEast' );
% Label axes
xlabel X1ss
ylabel Yajuste
grid on


