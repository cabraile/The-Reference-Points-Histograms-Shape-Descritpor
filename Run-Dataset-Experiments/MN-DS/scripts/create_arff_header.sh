#!/bin/bash

# GLOBAL PARAMETERS - CALLED ON 'main' METHOD
#####################

FILE_NAME=$1
NUM_ATTRIBUTES=$2
DATASET_PATH=$3
RELATION_NAME=$4

#####################

# AUXILIAR METHODS
#####################

write_relation()
{
  echo "@RELATION $1" >> $FILE_NAME
}

write_attributes()
{
  declare -i N
  declare -i i
  let "N=$1"
  let "i=1"
  while [ $i -le $N ]
  do
    echo "@ATTRIBUTE a_$i NUMERIC" >> $FILE_NAME
    let "i+=1"
  done
}

write_classes()
{
  local CLASSES_PATH=$1;
  echo -ne "@ATTRIBUTE class  {" >> $FILE_NAME
  # > Gets each class on the dataset
  declare -i INDEX
  let "INDEX=0"
  for CLASS in `ls $CLASSES_PATH`
  do
    if [ $INDEX -ge 1 ]
    then
      echo -ne "," >> $FILE_NAME
    fi
    echo -ne "$CLASS" >> $FILE_NAME
    let "INDEX+=1"
  done
  echo "}" >> $FILE_NAME
}

#####################

# MAIN FLOW
#####################

main()
{
  rm -rf $FILE_NAME
  write_relation $RELATION_NAME
  write_attributes $NUM_ATTRIBUTES
  write_classes $DATASET_PATH
  echo "@DATA" >> $FILE_NAME
}

#####################

main
