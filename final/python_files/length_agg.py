def build_cdf(model):
    cdf = []
    for i in range(len(lengths)):
        cdf.append([0] * len(lengths))
    for i in range(len(model)):
        cumsum = 0
        for connection in range(len(model[i])):
            cumsum += model[i][connection]
            cdf[i][connection] = cumsum
    return cdf


# full, half dot, half, quarter dot, quarter, eighth, sixteenth
lengths = ["16", "12", "8", "6", "4", "2", "1"]
lengths_model = []
for i in range(len(lengths)):
    lengths_model.append([0] * len(lengths))
prev_idx = None
matrix_start_state = input("paste in matrix start state")

if matrix_start_state:
    arr = matrix_start_state[2:-2].split("], [")
    for i in range(len(arr)):
        sp = arr[i].replace(" ", "").split(",")
        for j in range(len(sp)):
            lengths_model[i][j] = int(sp[j])
while True:
    length = input("Length: ").upper()
    if length in lengths:
        idx = lengths.index(length)
        if prev_idx is not None:
            lengths_model[prev_idx][idx] += 1
            print(f'    {lengths[prev_idx]} -> {length}: {lengths_model[prev_idx][idx]}')
        prev_idx = idx
    elif length == "QUIT":
        lengths_norm = [
            list(map(lambda x: 0 if sum(length) == 0 else x / sum(length), length))
            for length in lengths_model
        ]
        print(lengths_model)
        print("\n")
        print(lengths_norm)
        print("\n")
        print(build_cdf(lengths_norm))
        break
    else:
        print("invalid--skipped")