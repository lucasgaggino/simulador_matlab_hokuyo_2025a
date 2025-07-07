function resampled = resampleParticles(particles, map)
    N = size(particles,2);
    w = particles(4,:);
    w = w + eps;        
    w = w / sum(w);

    % Find indices of top 5 particles by weight (elitist selection)
    [~, sorted_indices] = sort(w, 'descend');
    top5_indices = sorted_indices(1:min(5, N));
    
    % Reserve space for top 5 particles + systematic resampling for the rest
    N_elite = min(5, N);
    N_resample = N - N_elite;
    
    % Initialize resampled array
    resampled = zeros(size(particles));
    
    % 1) Always keep top 5 particles (elitist selection)
    resampled(:, 1:N_elite) = particles(:, top5_indices);
    
    % 2) Systematic resampling for remaining particles
    if N_resample > 0
        cdf = cumsum(w);
        u0  = rand/N_resample;
        u   = u0 + (0:(N_resample-1))/N_resample;
        idx = arrayfun(@(ui) find(cdf>=ui,1), u);
        resampled(:, (N_elite+1):N) = particles(:, idx);
    end

    % 3) Rejuvenecimiento aumentado (10%) - but don't replace elite particles
    alpha = 0.1;
    M     = round(alpha * N_resample);  % Only apply to non-elite particles
    start_idx = N_elite + 1;
    for k = 1:M
        if (start_idx + k - 1) <= N
            resampled(:, start_idx + k - 1) = generar_particula(map);
        end
    end

    % 4) Jitter ligero para romper clones identicos - apply to all particles
    sigma_pos = 0.02;  sigma_ang = 0.01;
    
    % Apply jitter to elite particles (lighter jitter to preserve their quality)
    elite_sigma_pos = sigma_pos * 0.5;  % Reduced jitter for elite particles
    elite_sigma_ang = sigma_ang * 0.5;
    resampled(1:2, 1:N_elite) = resampled(1:2, 1:N_elite) + elite_sigma_pos*randn(2, N_elite);
    resampled(3, 1:N_elite)   = resampled(3, 1:N_elite)   + elite_sigma_ang*randn(1, N_elite);
    
    % Apply normal jitter to non-elite particles
    if N_resample > 0
        resampled(1:2, (N_elite+1):N) = resampled(1:2, (N_elite+1):N) + sigma_pos*randn(2, N_resample);
        resampled(3, (N_elite+1):N)   = resampled(3, (N_elite+1):N)   + sigma_ang*randn(1, N_resample);
    end

    % 5) Pesos uniformes para la siguiente iteracion
    %resampled(4,:) = 1/N;
end
