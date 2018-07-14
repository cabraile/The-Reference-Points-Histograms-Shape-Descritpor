#!/bin/bash

# PARAMETERS
################################

DS_PATH="/path/to/rgbd-dataset/"
# > Implement max number of parallel jobs
let "MAX_NUM_JOBS=1";
OUTPUT_PATH="/output/path/";

################################

# DEFAULTS
################################

SCRIPTS_PATH="scripts/";
ARFF_CACHE=".cache";
MAKE_DIR=".build";

################################

# MAIN FLOW
################################

DESCRIPTOR_TYPE="complete";
for Q in "27" "1"
do
    let "NB_OF_REFERENCE_POINTS=Q";
    if [ ${NB_OF_REFERENCE_POINTS} -ne 1 ]; then
        let "NUM_ATTRIB=3 * NB_OF_REFERENCE_POINTS";
    else
	let "NUM_ATTRIB=81"; # 3 * 27
    fi
    for SIZE in "10" "30" "50" "70" "100"
    do
      let "ARRAY_SIZE=SIZE";
      RELATION_NAME="INCRARR_${NB_OF_REFERENCE_POINTS}_${ARRAY_SIZE}_${DESCRIPTOR_TYPE}";
      WEKA_FILE_NAME="${OUTPUT_PATH}${RELATION_NAME}.arff";

      bash ${SCRIPTS_PATH}run_main_dataset.sh $DS_PATH $ARRAY_SIZE $MAX_NUM_JOBS $ARFF_CACHE $MAKE_DIR $NB_OF_REFERENCE_POINTS $DESCRIPTOR_TYPE;
      mkdir $OUTPUT_PATH ;
      bash ${SCRIPTS_PATH}create_arff_header.sh $WEKA_FILE_NAME $NUM_ATTRIB $DS_PATH $RELATION_NAME;
      bash ${SCRIPTS_PATH}concatenate_arff_files.sh $WEKA_FILE_NAME $ARFF_CACHE;
    done
done

################################
