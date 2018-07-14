#!/bin/bash

DS_PATH=$1
ARRAY_SIZE=$2
MAX_NUM_JOBS=$3
ARFF_CACHE=$4
MAKE_DIR=$5
NB_OF_REFERENCE_POINTS=$6
DESCRIPTOR_TYPE=$7

# > Keep checking if the number of jobs is not higher than MAX_NUM_JOBS
wait_max_jobs()
{
  # . Count the number of jobs
  declare -i JOBS_COUNT &&
  let "JOBS_COUNT=0";
  JOBS=`jobs -p`
  for JOB in $JOBS
  do
    let "JOBS_COUNT+=1";
  done
  while [ "$JOBS_COUNT" -ge "$MAX_NUM_JOBS" ]
  do
    # . Count the number of jobs
    let "JOBS_COUNT=0";
    JOBS=`jobs -p`
    for JOB in $JOBS
    do
      let "JOBS_COUNT+=1";
    done
  done
}

main()
{
  # > Dirs setup
  rm -rf $ARFF_CACHE ;
  mkdir $ARFF_CACHE ;
  mkdir ${ARFF_CACHE}/train ;
  mkdir ${ARFF_CACHE}/test ;
  mkdir $MAKE_DIR;

  # > Compilation
  cd $MAKE_DIR
  if [ ! -d "CMakeFiles" ]; then
    cmake .. ;
  fi
  make ;
  cd .. ;

  CLASSES=`ls $DS_PATH` &&
  for CLASS in $CLASSES
  do
    wait_max_jobs ;
    echo "==> Executing class $CLASS <==";

    # > Execute
    ${MAKE_DIR}/main -p $DS_PATH -c $CLASS -s $ARRAY_SIZE -o $ARFF_CACHE -n $NB_OF_REFERENCE_POINTS -t $DESCRIPTOR_TYPE &
  done

  wait ;
}

main ;
