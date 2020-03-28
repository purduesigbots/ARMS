#!/bin/bash
#Runs clang-format on all files (sans API.h) in a PROS repo
#Clang-format is needed on PATH
if [ -e "project.pros" ]
then
echo "Formatting Code..."
for i in ./include/*.h*
do
	if [ "${i##*/}" == "api.h" ]
	then
		continue; #ignore PROS files
	fi
	clang-format -i ./include/${i##*/}
done

for d in `find ./src -type d`;
do
	for i in ${d}/*.c*
	do
		clang-format -i ${d}/${i##*/}
	done
done
echo "Formatting Complete"
else
echo "Current directory is not a PROS project - will not format it."
fi
