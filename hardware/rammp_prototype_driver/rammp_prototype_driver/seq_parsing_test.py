test = "SEQ_STATUS,4,8,0\n"
split_string = test.split(",")
current_seq = split_string[1]
seq_length = split_string[2]
seq_mode = split_string[3].strip()
print(current_seq)
print(seq_length)
print(seq_mode)
