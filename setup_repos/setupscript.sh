#!/bin/bash
source ~/.bashrc

echo "Installing git repos"

files=(`grep -lHR repo*`)

for file in "${files[@]}"
do
	echo $file
done

#cd ../..
#echo ${arr[n]}
#if [ -d ""]; then
#    git clone 
#fi

#git clone 
