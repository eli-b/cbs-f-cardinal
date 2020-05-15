#!/bin/sh
echo "Running experiments"

name="Paris_1_256"
map="../instances/$name.map"
scen="../instances/scen-even/$name-even"
output="../exp/$name/$name-even"
time=60
for k in $(seq 50 10 200)
do
	for i in $(seq 1 1 25)
	do
		echo "$k agents on instance $name-even-$i"
		../build/cbs -m $map -a $scen-$i.scen -k $k -o $output-$k-CBS.csv -t $time -s 1 -h NONE -p 0
      		../build/cbs -m $map -a $scen-$i.scen -k $k -o $output-$k-CG.csv -t $time -s 1 -h CG
      		../build/cbs -m $map -a $scen-$i.scen -k $k -o $output-$k-CG-R.csv -t $time -s 1 -h CG --rectangleReasoning=1
      		../build/cbs -m $map -a $scen-$i.scen -k $k -o $output-$k-CG-R+C.csv -t $time -s 1 -h CG --rectangleReasoning=1 --corridorReasoning=1
      		../build/cbs -m $map -a $scen-$i.scen -k $k -o $output-$k-CG-R+C+T.csv -t $time -s 1 -h CG --rectangleReasoning=1 --corridorReasoning=1 --targetReasoning=1
      		../build/cbs -m $map -a $scen-$i.scen -k $k -o $output-$k-WDG-R.csv -t $time -s 1 -h WDG --rectangleReasoning=1
      		../build/cbs -m $map -a $scen-$i.scen -k $k -o $output-$k-WDG-R+C.csv -t $time -s 1 -h WDG --rectangleReasoning=1 --corridorReasoning=1
      		../build/cbs -m $map -a $scen-$i.scen -k $k -o $output-$k-WDG-R+C+T.csv -t $time -s 1 -h WDG --rectangleReasoning=1 --corridorReasoning=1 --targetReasoning=1
  	done
done



