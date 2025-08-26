# [PATHS]
#export HOMEFOLDER="$HOME/blockchain-framework-task-allocation/"
export HOMEFOLDER="$HOME/blockchain_robot_task_allocation_framework/"

export MAINFOLDER="$HOMEFOLDER"
export ARGOSFOLDER="$MAINFOLDER/argos-python"
export EXPERIMENTFOLDER="$MAINFOLDER/slam2dga"

# [FILES]
export ARGOSNAME="task-foraging"
export ARGOSFILE="${EXPERIMENTFOLDER}/experiments/${ARGOSNAME}.argos"
export ARGOSTEMPLATE="${EXPERIMENTFOLDER}/experiments/${ARGOSNAME}.x.argos"

# [ARGOS]
export NUM1=10 #number of low drift robots
export CON1="${EXPERIMENTFOLDER}/controllers/main_collab.py"
export NUM3=5 #number of high drift robots
export NUM2=0 #reserved number for other robot type
export NUMM=0

export RABRANGE="7.0"
export TPS=10
export DENSITY="1"

export NUMROBOTS=$(echo $NUM1+$NUM2+$NUM3 | bc)
export ARENADIM=$(echo "scale=3 ; sqrt(20/$DENSITY)" | bc)
export ARENADIMV=$(echo "scale=3 ; sqrt(20/$DENSITY)*3" | bc)
export ARENADIMH=$(echo "scale=3 ; $ARENADIM/2" | bc)
export ARENADIMHV=$(echo "scale=3 ; $ARENADIMV/2" | bc)
export STARTDIM=$(echo "scale=3 ; $ARENADIM/2" | bc)
export INTERLMDIST=$(echo "scale=3 ; $ARENADIM/4" | bc)
export NUMLANDMARKS="3"

#[Blockchain]
export BLOCKPERIOD=2

# [SC]
export MAXWORKERS=15
export LIMITASSIGN=2
export MAXCLUSTERS=5
export INITBALANCE=20
export MAXNEGCLUSTERS=5
export SAVECONFIRMATION=1 #0: reject confirmed cluster immediately for repeat estimation of the action distribution, 1: save confirmed cluster
export TASKREWARD=1 #0: TURN OFF TASK REWARD, as a baseline
export LOADEAPARAMS=1 #0: will not load parameters of epsilon-greedy agent, 1: load them


# [OTHER]
export SEED=1500
export TIMELIMIT=100
export LENGTH=10000
export SLEEPTIME=5
export REPS=1
export NOTES="Grand arena 3 landmarks experiment, with 1 malicious robot"




