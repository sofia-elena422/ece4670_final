# //  C4   262 Hz (middle C)   
# //  C4S  277
# //  D4   294
# //  D4S  311
# //  E4   330
# //  F4   349
# //  F4S  370
# //  G4   392
# //  G4S  415
# //  A4   440  
# //  A4S  466 
# //  B4   494
# //  C5   523  
# //  C5S  554  
# //  D5   587
# //  D5S  622
# //  E5   659
# //  F5   698
# //  F5S  740
# //  G5   784
# //  G5S  831
# //  A5   880
# //  A5S  932
def build_cdf(model):
    cdf = []
    for i in range(len(notes)):
        cdf.append([0]*len(notes))
    for i in range(len(model)):
        cumsum = 0
        for connection in range(len(model[i])):
            cumsum += model[i][connection]
            cdf[i][connection] = cumsum
    return cdf

notes = ["C4", "C4S","D4","D4S","E4","F4","F4S","G4","G4S","A4","A4S","B4","C5","C5S","D5","D5S","E5","F5","F5S","G5","G5S","A5","A5S"]
notes_model=[]
for i in range(len(notes)):
    notes_model.append([0]*len(notes))
prev_idx=None
matrix_start_state = input("paste in matrix start state")

if matrix_start_state:
    arr = matrix_start_state[2:-2].split('], [')
    for i in range(len(arr)):
        sp = arr[i].replace(' ','').split(',')
        for j in range(len(sp)):
            notes_model[i][j] = int(sp[j])
while True:
    note = input("Note: ").upper()
    if note in notes:
        idx = notes.index(note)
        if prev_idx is not None:
            notes_model[prev_idx][idx]+=1
        prev_idx = idx
    elif note == "QUIT":
        notes_norm = [list(map(lambda x: 0 if sum(note)==0 else x/sum(note),note)) for note in notes_model]
        print(notes_model)
        print('\n')
        print(notes_norm)
        print('\n')
        print(build_cdf(notes_norm))
        break
    else:
        print("invalid--skipped")