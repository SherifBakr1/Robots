import pandas as pd

# Load Q-tables from CSV files
q_table_forward = pd.read_csv('episode1600_left_turn_working.csv', index_col=0)
q_table_left_turn = pd.read_csv('600.csv', index_col=0)
#q_table_u_turn = pd.read_csv('q_table_u_turn.csv', index_col=0)

# Combine Q-tables by taking the maximum Q-value for each state-action pair
consolidated_q_table = pd.DataFrame(index=q_table_forward.index)
for action in q_table_forward.columns:
    consolidated_q_table[action] = q_table_forward[action].combine(q_table_left_turn[action], max)

# for action in q_table_forward.columns:
#     consolidated_q_table[action] = q_table_forward[action].combine(q_table_left_turn[action], max).combine(q_table_u_turn[action], max)
# Save the consolidated Q-table to a new CSV file
consolidated_q_table.to_csv('consolidated_q_table.csv')