function [windFileOut, W]=Af_MakeWind(fast,Disturbance,Simulation,varargin)
% Inputs:  fast. struct
%          Disturbance. struct
%                     .Type     - type of wind input, options: const, steps,
%                     ramp, sine, step, EOG, ECD
%                     .U_ref    - initial (or constant) wind before transient
%                     .LHshear  - linear horizontal wind shear
%                     .Vshear   - vertical (exponential) shear
%                     .LVshear  - linear vertical shear
%                     .TStart   - time to start transient event
%                     .TEnd     - time to end transient event
%                     .Step     - wind speed step size
%                     .Amp      - amplitude (for sine)
%                     .Per      - period (for sine and EOG)
%          Simulation struct
%
%  DZalkind 6/26/2020 - cleaned up old script, only const and step tested

%% Input Handling
if ~isempty(varargin) > 0
    Mplot = varargin{1};
end


% Simulation parameters
if isfield(Simulation,'TMax')
    TMax = Simulation.TMax;
else
    TMax = 9000;
end

% Disturbance.Type
if isfield(Disturbance,'Type')
    TYPE    = Disturbance.Type;
else
    disp('Af_MakeWind Warning: no disturbance type, setting constant wind input')
    TYPE    = 'const';
end

% Disturbance DT is usually 0.05 sec, but we can set it
if ~isfield(Disturbance,'DT')
    DT = 0.05;
else
    DT = Disturbance.DT;
end


%% Type Input Handling

% U_ref is needed for all
if ~isfield(Disturbance,'U_ref')
    Disturbance.U_ref = 15;
    disp(['Af_MakeWind Warning: no U_ref set for ', TYPE, ' step wind input, setting U_ref = 15 m/s']);
end    
    
if strcmp(TYPE,'step') % Single Step in Wind
    
    if ~isfield(Disturbance,'Step')
        Disturbance.Step = 2;
        disp('Af_MakeWind Warning: no Step set for step wind input, setting Step = 2 m/s');
    end
    
    if ~isfield(Disturbance,'TStart')
        Disturbance.TStart = 100;
        disp('Af_MakeWind Warning: no TStart set for step wind input, setting TStart = 100 s');
    end
    
elseif strcmp(TYPE,'ramp')  % Ramp wind going from Vmin to Vmax to min, ramp starts at Stime
    if ~isfield(Disturbance,'U_max')
        Disturbance.Umin = 3;
        disp('Af_MakeWind Warning: no U_max set for ramp wind input, setting Umin = 3 m/s');
    end
    
    
    if ~isfield(Disturbance,'TStart')
        Disturbance.TStart = 100;
        disp('Af_MakeWind Warning: no TStart set for ramp input, setting TStart = 100 s');
    end
    
    if ~isfield(Disturbance,'TEnd')
        Disturbance.TEnd = 1100;
        disp('Af_MakeWind Warning: no TEnd set for ramp input, setting TEnd = 1100 s');
    end
    
    
elseif strcmp(TYPE,'sine')   % Sinusoidal wind of DC+amp*sin(2*pi*freq*t)
    
    if ~isfield(Disturbance,'Amp')
        Disturbance.Amp = 2;
        disp('Af_MakeWind Warning: no Amp set for sine wind input, setting Amp = 2 m/s');
    end
    
    if ~isfield(Disturbance,'Per')
        Disturbance.Per = 30;
        disp('Af_MakeWind Warning: no Per set for sine wind input, setting Per = 30 sec');
    end
    
elseif strcmp(TYPE,'steps')    % Multiple steps in wind from Umin to Umax
    if ~isfield(Disturbance,'Umin')
        Disturbance.Umin = 3;
        disp('Af_MakeWind Warning: no Vmin set for steps wind input, setting Umin = 3 m/s');
    end
    
    if ~isfield(Disturbance,'Umax')
        Disturbance.Umax = 25;
        disp('Af_MakeWind Warning: no Umax set for steps wind input, setting Umax = 25 m/s');
    end
    
    if ~isfield(Disturbance,'TStart')
        Disturbance.TStart = 100;
        disp('Af_MakeWind Warning: no TStart set for steps input, setting TStart = 100 s');
    end
    
    if ~isfield(Disturbance,'TEnd')
        Disturbance.TEnd = 1100;
        disp('Af_MakeWind Warning: no TEnd set for steps input, setting TEnd = 1100 s');
    end
    
    
elseif strcmp(TYPE,'EOG')
    if ~isfield(Disturbance,'TStart')
        Disturbance.TStart = 100;
        disp('Af_MakeWind Warning: no TStart set for EOG input, setting TStart = 100 s');
    end
    
    if ~isfield(Disturbance,'Per')
        Disturbance.Per = 30;
        disp('Af_MakeWind Warning: no Per set for EOG wind input, setting Per = 30 sec');
    end
    
elseif strcmp(TYPE,'ECD')
    if ~isfield(Disturbance,'TStart')
        Disturbance.TStart = 100;
        disp('Af_MakeWind Warning: no TStart set for ECD input, setting TStart = 100 s');
    end
    
    if ~isfield(Disturbance,'TEnd')
        Disturbance.TEnd = 100;
        disp('Af_MakeWind Warning: no TEnd set for EOG input, setting TEnd = 110 s');
    end
    
end

%% Initialize W matrix for .wnd input
% The following parameters are fully described in the TurbSim/Aerodyn docs.

% TMax= Max time, DT=Time step, WD= Wind Direction, VS= Vertical speed
% Hshear=Linear horizontal shear, Vshear= Power Law Vertical Shear,
% LVshear= Linear Vertical Shear, GS= Gust Speed
% Mplot==1 will produce a plot of the generated wind file (hub height)

% Note: If any shear parameter input is a scalar, it will produce a wind
% file with a constant shear.  If the input shear parameter is a vector,
% it must be the same length as the generated wind file and is used as the
% time varying shear.

% EXAMPLE with Header
% ! Time	Wind	Wind	Vert.	Horiz.	Vert.	LinV	Gust
% !	        Speed	Dir	    Speed	Shear	Shear	Shear	Speed
%   0.0	     30	    0       -1      0.1     0.14	0       0

% Generated wind file is stored in the PWD\WindData folder with
% the appropriate name for the type.

% Initialize
samples     = round(TMax/DT+1);
W           = zeros(samples, 8);

% Time
W(:,1)      = 0:DT:TMax;

% Direction, set to 0 unless ECD
W(:,3)      = 0;

% Vertical Speed, none for now
W(:,4)      = 0;

% Horizontal Shear (linear)
if isfield(Disturbance, 'LHshear')
    W(:,5) = Disturbance.LHsear;
else
    W(:,5) = 0;
end

% Vertical Shear (exponential)
if isfield(Disturbance, 'Vshear')
    W(:,6) = Disturbance.Vshear;
else
    W(:,6) = 0;
end

% Linear Vertical Shear
if isfield(Disturbance, 'LVshear')
    W(:,7) = Disturbance.LVshear;
else
    W(:,7) = 0;
end

% Gust, set to 0 for now, let wind speed do the work
W(:,8)      = 0;



%% Make W(:,2), Wind Speed Vector

switch TYPE
    case 'const'  % p1=DC VALUE
        W(:,2) = ones(samples,1)*Disturbance.U_ref;
        
    case 'step'
        tt = W(:,1);
        uu = Disturbance.U_ref * ones(size(tt));
        uu(tt>Disturbance.TStart) = Disturbance.U_ref + Disturbance.Step;
        W(:,2) = uu;
        
        
    case 'steps' %p1=min, p2=max, p3=ds
        
%         p2=p2+p3;
%         
%         sgn=sign(p2-p1);
%         
%         W(:,2)=ones(samples,1)*p1;
%         steptime=round(samples/2/(sgn*(p2-p1)/p3));
%         
%         for n=1:2*sgn*(p2-p1)/p3
%             b=(n-1)*steptime+1;
%             e=n*steptime+1;
%             if e>samples;
%                 e=samples;
%             end
%             if n<=sgn*(p2-p1)/p3
%                 W(b:e,2)=p1+sgn*(n-1)*p3;
%             end
%             if n>sgn*(p2-p1)/p3
%                 W(b:e,2)=p2-sgn*(n-sgn*(p2-p1)/p3)*p3;
%             end
%         end
        
        
    case 'sine' % p1=dcwind, p2=freq, p3=amp,
        W(:,2)= Disturbance.U_ref + Disturbance.Amp*sin(2 * pi * W(:,1) / Disturbance.Period);
        
    case 'ramp' % p1=min, p2=max, p3=ST
        
        % ramp time breakpoints
        t_ramp  = [Disturbance.TStart,  (Disturbance.TStart + Disturbance.TEnd)/2,  Disturbance.TEnd];
        v_ramp  = [Disturbance.U_ref,   Disturbance.U_max,                          Disturbance.U_ref];
        
        % append start end
        t_ramp  = [0,                   t_ramp,     Simulation.TMax];
        v_ramp  = [Disturbance.U_ref,   v_ramp,     Disturbance.U_ref];
        
        % catch non-unique t_ramp(s)
        [t_ramp, iu]    = unique(t_ramp);
        v_ramp          = v_ramp(iu);
        
        W(:,2)  = interp1(t_ramp',v_ramp',W(:,1));
        
        
        
%     case 'yaw_step' % p1:5  [ Wind Speed    Initial Yaw     Start Time      Final Yaw       End Time ]
%         
%         tt = W(:,1);
%         
%         if p3 == p5     %just a step
%             yy = p2*ones(samples,1);
%             yy(tt>p3) = p4;
%             
%         else            %ramp
%             X = [0,p3,p5,TMax];
%             V = [p2,p2,p4,p4];
%             yy = interp1(X,V,tt);
%         end
%         W(:,2) = ones(samples,1)*p1./cosd(yy)+1.373157942907710e-08;
%         W(:,2) = ones(samples,1)*p1;
%         W(:,3) = yy;
%         
        
    case 'EOG'
        p3 = Disturbance.Per;
        p2 = Distrubance.TStart;
        p1 = Disturbance.U_ref;
        
        tt = W(:,1);
        tt_gust = (0:DT:p3)';
        U  = p1;
        gust = [zeros(p2/DT,1);
            -0.37*p4*sin(3*pi*tt_gust/p3).*(1-cos(2*pi*tt_gust/p3));
            zeros(samples-p2/DT-p3/DT-1,1)];
        W(:,2) = U+gust;
        
    case 'ECD'
        p1 = Disturbance.U_ref;
        p2 = Disturbance.TStart;
        p3 = Distrubance.TEnd;
        p4 = Disturbance.U_ref + 15;
    
        W(1:samples,2)=p1;
        W((p2/DT)+1:((p2+p4)/DT)+1,2)= p1+0.5*15*(1-cos(pi*[((0)/DT)+1:((p4)/DT)+1]/(p4/DT)));
        W(round((p2+p4)/DT)+1:end,2)= p3;
        if p1<4
            theta_cg=180;
        else
            theta_cg=720/p1;
        end
        W(1:samples,3) = 0;
        W((p2/DT)+1:((p2+p4)/DT)+1,3) = 0.5*theta_cg*(1-cos(pi*[((0)/DT)+1:((p4)/DT)+1]/(p4/DT)));
        W(round((p2+p4)/DT)+1:end,3)= theta_cg;
        
        
    case 'PeakTest'
        startTime = 200;                    %start time of first gust
        extraTime = 100;                    %time after test
        periods = [60,45,30,20,10,5];       %sinusoid periods
        
        DT = 1/80;
        T = periods(1);
        tt = 0:DT:TMax;
        gust = zeros(length(periods),length(tt));
        
        startGust = [startTime,startTime + cumsum(periods(1:end-1))];
        
        Amp = 6;                            %gust amplitude
        startU = 9.3;                       %wind speed to start gust
        
        for iGust = 1:size(gust,1)-1
            T = periods(iGust);
            gust(iGust,:) = (Amp/2 * (-cos(2*pi/T * (tt - startGust(iGust)))+1)).*(tt >= startGust(iGust) & tt <= startGust(iGust+1));
        end
        
        wind = sum(gust,1) + startU;
        W(:,1) = tt;
        W(:,2) = wind;
        
        if 1
            figure(100);
            plot(tt,wind);
        end
        
    case 'ConstRake' % p1=min, p2=t_start, p3=t_end, p4 = du
        
        tt      = 0:DT:TMax;
        b       = p1 - p4/20 * p2;
        m       = p4 /20;
        maxWS   = 25;
        tend    = (maxWS-b)*(20/p4);
        
        W(:,2)      = p1;
        W(tt>p2,2)  = m * tt(tt>p2) + b;
        W(tt>tend,2)    = maxWS;
        
        if 1
            figure(100);
            plot(tt,W(:,2));
        end
end


%% Write Output
if ~isdir(fast.FAST_runDirectory)
    mkdir(fast.FAST_runDirectory);
end

windFileOut = fullfile(fast.FAST_runDirectory,[fast.FAST_namingOut,'.wnd']);
dlmwrite(windFileOut, W, 'delimiter','\t','precision','%.6f')
disp([TYPE,' Wind File Made: ',fast.FAST_namingOut,'.wnd'])

if Mplot==1
    figure(800);
    set(gcf,'Name','Wind Input')
    plot(W(:,1),W(:,2))
    xlabel('Time [s]')
    ylabel('HH Velocity [m/s]')
    grid on
end