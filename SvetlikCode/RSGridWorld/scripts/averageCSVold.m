function averageCSVold
path = '/home/users/max/Documents/programovani/burlapcustomdomains/RSGridWorld/data/';
steps_trial = 1500;
epochs = 100;
trials = 10;

function [M_mean,e] = cumstep_avg(filename)
    fullpath = strcat(path, filename);
    M = csvread(fullpath,1,1);
    M_mean = zeros(steps_trial,3);
    M_mean(:,1) = 1:steps_trial;
    size = epochs*trials;
    for step = 1:steps_trial
       M_mean(step,2) = mean(M(step:steps_trial:size*steps_trial,3));
       sdev = std(M(step:steps_trial:end,3))/sqrt(size);
       M_mean(step,3) = sdev * tinv(0.995,size); %99 conf int
    end
end
    function fire_weak
    %Training: 500steps at -5014 rwd cumulative
    training_offset = 0;
    fire_baseline = cumstep_avg('fire4_baseline.csv');
    fire_pot = cumstep_avg('fire4_fwd_advice_gamma0.0.csv');
    fire_g1 = cumstep_avg('fire4_fwd_advice_gamma0.1.csv');
    fire_g2 = cumstep_avg('fire4_fwd_advice_gamma0.3.csv');
    fire_g3 = cumstep_avg('fire4_fwd_advice_gamma0.5.csv');
    fire_g4 = cumstep_avg('fire4_fwd_advice_gamma0.7.csv');
    fire_g5 = cumstep_avg('fire4_fwd_advice_gamma0.9.csv');

    x = fire_baseline(:,1);
    colors = ['b','r','y','m','c','g','k','w']; %hacky workaround for legend colors to match that of shadedErrorBar

    hold on
    plot(x, fire_baseline(:,2), training_offset+x, fire_pot(:,2), training_offset+x,fire_g1(:,2), training_offset+x,fire_g2(:,2), ...
            training_offset+x, fire_g3(:,2),training_offset+x, fire_g4(:,2),training_offset+x, fire_g5(:,2));
    bsl = shadedErrorBar(x,fire_baseline(:,2),fire_baseline(:,3),colors(1),1);
    pot = shadedErrorBar(training_offset+x,fire_pot(:,2),fire_pot(:,3),colors(2),1);
    fwd = shadedErrorBar(training_offset+x,fire_g1(:,2),fire_g1(:,3),colors(3),1);
    fwd = shadedErrorBar(training_offset+x,fire_g2(:,2),fire_g2(:,3),colors(4),1);
    fwd = shadedErrorBar(training_offset+x,fire_g3(:,2),fire_g3(:,3),colors(5),1);
    fwd = shadedErrorBar(training_offset+x,fire_g4(:,2),fire_g4(:,3),colors(6),1);
    fwd = shadedErrorBar(training_offset+x,fire_g5(:,2),fire_g5(:,3),colors(7),1);

    title('Potential Shaped Transfer Expected Reward For the Fire4 Task')
    xlabel('Steps')
    ylabel('Expected Reward')
    legend('fire baseline', 'fire gamma 0.0', 'fire gamma 0.1','fire gamma 0.3','fire gamma 0.5','fire gamma 0.7','fire gamma 0.9','Location','northwest');
    end

    function pit_weak
    %Training: 500 steps at -13036 cum rwd
    training_offset = 700;
    fp_baseline = cumstep_avg('pit4_baseline.csv');
    fp_avg = cumstep_avg('pit4_no_fwd_advice.csv');
    pit_fwd = cumstep_avg('pit4_fwd_advice.csv');

    x = fp_baseline(:,1);
    colors = ['b','r','y'];
    hold on
    plot(x, fp_baseline(:,2), training_offset+x, fp_avg(:,2),training_offset+x,pit_fwd(:,2));
    bsl = shadedErrorBar(x,fp_baseline(:,2),fp_baseline(:,3),colors(1),1);
    pot = shadedErrorBar(training_offset+x,fp_avg(:,2),fp_avg(:,3),colors(2),1);
    fwd = shadedErrorBar(training_offset+x,pit_fwd(:,2),pit_fwd(:,3),colors(3),1);

    title('Potential Shaped Transfer Expected Reward For the Pit4 Task')
    xlabel('Steps')
    ylabel('Expected Reward')
    h=legend('pit baseline', 'pit with potential', 'pit with forward advice','Location','northwest');
    end
    
    function nstep
    %Training: 500 steps on each subtask ??? cum rwd
    %2 subtasks
    training_offset = 0;
    fp_baseline = cumstep_avg('target5_baseline.csv');
    fp_avg = cumstep_avg('target5_fire2_pit2_goal5_average20.7.csv');
    fp_min = cumstep_avg('target5_fire2_pit2_goal5_min20.7.csv');
    fp_max = cumstep_avg('target5_fire2_pit2_goal5_max20.7.csv');

    x = fp_baseline(:,1);
    colors = ['b','r','y','m'];
    hold on
    plot(x, fp_baseline(:,2), training_offset+x, fp_avg(:,2),training_offset+x, fp_min(:,2),training_offset+x, fp_max(:,2));
    bsl = shadedErrorBar(x,fp_baseline(:,2),fp_baseline(:,3),colors(1),1);
    avg = shadedErrorBar(training_offset+x,fp_avg(:,2),fp_avg(:,3),colors(2),1);
    max = shadedErrorBar(training_offset+x,fp_min(:,2),fp_min(:,3),colors(3),1);
    min = shadedErrorBar(training_offset+x,fp_max(:,2),fp_max(:,3),colors(4),1);

    title('Potential Function Merging Heuristics for 2 step Target6 Transfer')
    xlabel('Steps')
    ylabel('Cumulative Reward')
    h=legend('baseline', 'sum', 'min','max','Location','northwest');
    end

    function target5
    %Training: 500 steps on each subtask ??? cum rwd
    %2 subtasks
    %train = 2500;
    %steps_trial = 1500;
    %old stuff below
    %{
    baseline = cumstep_avg('target5_baseline.csv');
    %fire = cumstep_avg('target5_fire9_pit9_goal9_average_nogoal40.7.csv');
    fire = cumstep_avg('target5_fire2_pit2_goal90.7.csv');
    %pit = cumstep_avg('target5_fire9_pit9_goal9_average0.7.csv');
    pit = cumstep_avg('target5_fire2_pit2_goal9_into_goal40.7.csv');
    %music = cumstep_avg('target5_fire9_pit9_goal9_average_goalconv0.7.csv');
    music = cumstep_avg('target5_fire2_pit2_goal9_goal4separate0.7.csv');
    %music = cumstep_avg('target5_goal4_goal6_goal5_averagebs0.7.csv');
    %}
    train = 0;
    steps_trial = 1500;
    baseline = cumstep_avg('target5_baseline_rwd200.csv');
    fire = cumstep_avg('target5_tree_p2_f2_g7_1000_rwd200_0.7.csv');
    pit = cumstep_avg('target5_tree_p2_f2_g78_1500_rwd200_0.7.csv');
    music = cumstep_avg('target5_tree_p2_f2_g789_2000_rwd200_0.7.csv');

    x = baseline(:,1);
    x = 10*x;
    colors = ['b','r','g','m'];
    hold on
    
    %Note that the 'old stuff' had train, train+1000, train+1500 amounts of training
    %respectively
    density = 20;
    plot(downsample(x, density), downsample(baseline(:,2), density),'b^','MarkerSize', 12);
    plot(downsample(x+800, density), downsample(fire(:,2),density),'ro','MarkerSize', 12);
    plot(downsample(x+1200, density), downsample(pit(:,2),density),'gx','MarkerSize', 12);
    plot(downsample(x+2000, density), downsample(music(:,2), density),'ms','MarkerSize', 12);
    bsl = shadedErrorBar(x,baseline(:,2),baseline(:,3),colors(1),1);
    avg = shadedErrorBar(x+800,fire(:,2),fire(:,3),colors(2),1);
    max = shadedErrorBar(x+1200,pit(:,2),pit(:,3),colors(3),1);
    min = shadedErrorBar(x+2000,music(:,2),music(:,3),colors(4),1);

    %title('Policy Evaluation in a 10x10 Gridworld Task')
    xlabel('Steps Spent Learning in Target')
    ylabel('Expected Reward')
    %h=legend('No Transfer', 'Fire,Pit,1x10 Goal', 'Fire,Pit,Goal: 1x10->4x4','Fire,Pit,Goal:1x10,4x4','Location','northwest');
    h=legend('No Transfer', 'Fire,Pit,1x5 Goal', 'Fire,Pit,Goal: 1x5, 1x7','Fire,Pit,Goal:1x5,1x7,1x10','Location','northwest');
    end


    function target10
    %Training: 500 steps on each subtask ??? cum rwd
    %2 subtasks
    steps_trial = 30000;
    training_offset = 0;
    baseline = cumstep_avg('target10_baseline.csv');
    fire = cumstep_avg('fire2_to_target10_gamma0.25.csv'); %17 episodes
    pit = cumstep_avg('pit2_to_target10_gamma0.25.csv');
    music = cumstep_avg('goal5_to_target10_gamma0.7.csv');
    %music = cumstep_avg('target5_fire2_pit2_goal5_average0.25.csv');
    %music = cumstep_avg('target5_goal4_goal6_goal5_average0.7.csv'); %50 training
    x = baseline(:,1);
    colors = ['b','r','y','m'];
    hold on
    
    plot(x, baseline(:,2), 17+x, fire(:,2),35+x, pit(:,2),50+x, music(:,2));
    bsl = shadedErrorBar(x,baseline(:,2),baseline(:,3),colors(1),1);
    avg = shadedErrorBar(17+x,fire(:,2),fire(:,3),colors(2),1);
    max = shadedErrorBar(35+x,pit(:,2),pit(:,3),colors(3),1);
    min = shadedErrorBar(50+x,music(:,2),music(:,3),colors(4),1);

    title('Three Step Transfer into the Target10 Task')
    xlabel('Episodes')
    ylabel('Reward')
    h=legend('Baseline','Fire2', 'Pit2','Goal5','Location','northwest');
    end

%fire_weak;
%pit_weak;

%nstep;
%target10;
target5;
end