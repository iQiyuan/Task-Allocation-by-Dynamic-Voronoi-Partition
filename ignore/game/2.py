def u1(t, a1, a2):
    return t * a1 - (a2 ** 2)

def u2(t, a1, a2):
    return t * a2 - a1

def solve_dynamic_game(T, action_set, u1_func, u2_func):
    V1 = {}
    V2 = {}
    optimal_actions = {}

    t = T
    payoff_matrix = {}
    for a1 in action_set:
        for a2 in action_set:
            payoff_matrix[(a1, a2)] = (u1_func(t, a1, a2), u2_func(t, a1, a2))

    best_responses_p1 = {}
    best_responses_p2 = {}

    for a2 in action_set:
        max_u1 = max(payoff_matrix[(a1, a2)][0] for a1 in action_set)
        best_responses_p1[a2] = [a1 for a1 in action_set if payoff_matrix[(a1, a2)][0] == max_u1]

    for a1 in action_set:
        max_u2 = max(payoff_matrix[(a1, a2)][1] for a2 in action_set)
        best_responses_p2[a1] = [a2 for a2 in action_set if payoff_matrix[(a1, a2)][1] == max_u2]

    nash_equilibria = []
    for a1 in action_set:
        for a2 in action_set:
            if a1 in best_responses_p1[a2] and a2 in best_responses_p2[a1]:
                nash_equilibria.append((a1, a2))

    a1_star, a2_star = nash_equilibria[0]
    V1[t] = payoff_matrix[(a1_star, a2_star)][0]
    V2[t] = payoff_matrix[(a1_star, a2_star)][1]
    optimal_actions[t] = (a1_star, a2_star)

    for t in range(T - 1, 0, -1):
        payoff_matrix = {}
        next_a1_star, next_a2_star = optimal_actions[t + 1]
        future_v1 = V1[t + 1]
        future_v2 = V2[t + 1]

        for a1 in action_set:
            for a2 in action_set:
                cur_u1 = u1_func(t, a1, a2)
                cur_u2 = u2_func(t, a1, a2)
                total_u1 = cur_u1 + future_v1
                total_u2 = cur_u2 + future_v2
                payoff_matrix[(a1, a2)] = (total_u1, total_u2)

        best_responses_p1 = {}
        best_responses_p2 = {}

        for a2 in action_set:
            max_u1 = max(payoff_matrix[(a1, a2)][0] for a1 in action_set)
            best_responses_p1[a2] = [a1 for a1 in action_set if payoff_matrix[(a1, a2)][0] == max_u1]

        for a1 in action_set:
            max_u2 = max(payoff_matrix[(a1, a2)][1] for a2 in action_set)
            best_responses_p2[a1] = [a2 for a2 in action_set if payoff_matrix[(a1, a2)][1] == max_u2]

        nash_equilibria = []
        for a1 in action_set:
            for a2 in action_set:
                if a1 in best_responses_p1[a2] and a2 in best_responses_p2[a1]:
                    nash_equilibria.append((a1, a2))

        a1_star, a2_star = nash_equilibria[0]
        V1[t] = payoff_matrix[(a1_star, a2_star)][0]
        V2[t] = payoff_matrix[(a1_star, a2_star)][1]
        optimal_actions[t] = (a1_star, a2_star)

    return optimal_actions, V1, V2


T = 3
action_set = [0, 1]
optimal_actions, V1, V2 = solve_dynamic_game(T, action_set, u1, u2)
print("Optimal Strategies:")
for t in range(1, T + 1):
    print(f"Stage {t}: P1 -> {optimal_actions[t][0]}, P2 -> {optimal_actions[t][1]}")
print("Values:")
for t in range(1, T + 1):
    print(f"From stage {t}: U1 = {V1[t]}, U2 = {V2[t]}")
print(f"Total from start: U1 = {V1[1]}, U2 = {V2[1]}")
