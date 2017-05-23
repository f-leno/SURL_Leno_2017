%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% James Edwards 15 Sep 2009
% Plot a region with a bold line in the middle and a lighter colour around
% the region. Used for a mean/std dev thing originally but useful for
% others I suppose.
% x: x-axis
% mean: the line to draw. these are y values corresponding to the x values.
% upper: upper level of region. y values corr. to x values
% lower: lower level of region. "" ""
% colour: basic colour to use.
% uses row vectors
%%%%%%% TO DO
% 1. check if its a column vector and then use flipud
% 2. get rid of the black border of the regions
% 3. allow alpha to be an optional argument
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function confi(x,mean,upper,lower,colour)
    if nargin~=5
        error('usage: confi(x,mean,upper,lower,colour)');
    end

    if length(x)~=length(mean) || length(mean)~=length(lower) || length(lower)~=length(upper);
        error('mean,lower&upper vectors give y-values corresponding to the x values. Hence they must all be the same length!')
    end
    held = ishold;
    if ~held
        hold(gca,'on')
    end
    xv = [x,fliplr(x)];
    yv = [upper,fliplr(lower)];
    hReg = fill(xv,yv,colour); % draw region
    set(get(get(hReg,'Annotation'),'LegendInformation'),'IconDisplayStyle','off'); % Exclude line from legend
    alpha(0.3); % transparency for overlays
    plot(x,mean,'Color',colour, 'Linewidth', 2) % mean
    if ~held
        hold(gca,'off')
    end
end