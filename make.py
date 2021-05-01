import subprocess
import time
import os

paths = []

for root, dirs, files in os.walk("Instances"):
    for file in files:
        if file.endswith(".txt"):
             #print(os.path.join(root, file))
             paths.append(os.path.join(file))

paths.sort()

excluded = ['FLP-100-40-1.txt', 'FLP-150-45-1.txt', 'FLP-150-60-0.txt', 'FLP-150-60-1.txt', 'FLP-150-60-2.txt',
'FLP-200-60-0.txt', 'FLP-200-60-1.txt', 'FLP-200-60-2.txt', 'FLP-200-80-0.txt', 'FLP-200-80-1.txt', 'FLP-200-80-2.txt', 'FLP-250-75-0.txt', 'FLP-250-75-1.txt', 'FLP-250-75-2.txt',
'FLP-250-100-2.txt', 'FLP-250-100-1.txt', 'FLP-250-100-0.txt']

results = open("local_search_solutions.txt", "w")
# results = open("VND_tei_2.txt", "w")
#for filename in paths[21:]:
for filename in excluded[4:]:
	# if filename in excluded:
	# 	pass
	# else:
	start = time.time()
	print(filename)
	proc = subprocess.call(["python","flp.py",filename], stdout=results)
	end = time.time()
	# if filename=='FLP-100-40-1.txt' or filename=='FLP-150-45-1.txt' or filename=='FLP-150-60-0.txt' or filename=='FLP-150-60-1.txt'or filename=='FLP-150-60-2.txt' or filename=='FLP-200-60-0.txt' or filename=='FLP-200-60-1.txt' or filename=='FLP-200-60-2.txt' or filename=='FLP-200-80-0.txt' or filename=='FLP-200-80-1.txt' or filename=='FLP-200-80-2.txt':
	# 	pass
	# elif filename=='FLP-250-100-0.txt' or filename=='FLP-250-100-1.txt' or filename=='FLP-250-100-2.txt' or filename=='FLP-250-75-0.txt' or filename=='FLP-250-75-1.txt' or filename=='FLP-250-75-2.txt':
	# 	pass
	# else:
	# start = time.time()
	# print(filename)
	# proc = subprocess.call(["python","flp.py",filename], stdout=results)
	# end = time.time()
	# if end-start>10*60:
	# 	print(end-start, "superior to 10 min")
	# 	break
# 	end = time.time()
# 	print(end - start)
# 	start = time.time()
# end = time.time()
# print(end - start)