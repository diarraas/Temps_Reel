/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBATTERY 20
#define PRIORITY_TKILL 30


/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_openComRobot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_modeWD, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_errorCmpt, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
      
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_kill, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
     if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_killServer, "th_killServer", 0, PRIORITY_TKILL, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);

    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_killServer, (void(*)(void*)) & Tasks::KillServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}



/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status, err;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    cout << "Got server barrier " << endl << flush;
    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    /*
    rt_mutex_acquire(&mutex_readySend, TM_INFINITE);
    readySend = 1 ;
    rt_mutex_release(&mutex_readySend);
    
    rt_mutex_acquire(&mutex_readyRcv, TM_INFINITE);
    readySend = 1 ;
    rt_mutex_release(&mutex_readyRcv);
    */
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
        
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;
    while (1) {
            msgRcv = monitor.Read();
            cout << "Rcv <= " << msgRcv->ToString() << endl << flush;
            if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
                rt_sem_v(&sem_kill);
                rt_sem_p(&sem_serverOk, TM_INFINITE);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
                rt_sem_v(&sem_openComRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
                rt_mutex_acquire(&mutex_modeWD, TM_INFINITE);
                modeWD = 0 ;
                rt_mutex_release(&mutex_modeWD);
                rt_sem_v(&sem_startRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
                rt_mutex_acquire(&mutex_modeWD, TM_INFINITE);
                modeWD = 1 ;
                rt_mutex_release(&mutex_modeWD);
                rt_sem_v(&sem_startRobot);
            } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                    msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msgRcv->GetID();
                rt_mutex_release(&mutex_move);
            }
            delete(msgRcv); // mus be deleted manually, no consumer
        }
    }        

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            rt_mutex_acquire(&mutex_openComRobot, TM_INFINITE);
            openComRobot = 0;
            rt_mutex_release(&mutex_openComRobot);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            rt_mutex_acquire(&mutex_openComRobot, TM_INFINITE);
            openComRobot = 1;
            rt_mutex_release(&mutex_openComRobot);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
    
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    cout << "BARRIER FOR " << __PRETTY_FUNCTION__ << endl << flush;
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    int rs, wd, cptWD,cmpt = 0 ;
  //  rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        rt_mutex_acquire(&mutex_modeWD, TM_INFINITE);
        wd = modeWD;
        rt_mutex_release(&mutex_modeWD);
        if(wd == 0){
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
        }else{
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_task_set_periodic(NULL, TM_NOW, 1000000000);
            rt_mutex_release(&mutex_robot);        
        }
        
        cout << msgSend->GetID();
        cout << ")" << endl;
        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon
            
        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rs = robotStarted;
                rt_mutex_release(&mutex_robotStarted);
                rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
                errorCmpt = 0  ;
                cmpt = 0;
                rt_mutex_release(&mutex_errorCmpt);
        }else if(msgSend->CompareID(MESSAGE_ANSWER_NACK) || msgSend->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT) ||
               msgSend->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND) || msgSend->CompareID(MESSAGE_ANSWER_ROBOT_ERROR) || msgSend->CompareID(MESSAGE_ANSWER_COM_ERROR)){
                rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
                errorCmpt += 1 ;
                cmpt = errorCmpt;
                rt_mutex_release(&mutex_errorCmpt);
                if(cmpt == 3){
                    cout << "COM LOST "<<endl<<flush;
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(new Message(MESSAGE_ANSWER_COM_ERROR));
                    rt_mutex_release(&mutex_monitor);
                }
        }
        
        cptWD = 0;
      //  cout << "MODE" << modeWD << endl << flush;
        while(wd == 1 && rs == 1){
        //    cout << "IN WATCHDOG LOOP" << endl << flush;
            rt_task_wait_period(NULL);
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(new Message(MESSAGE_ROBOT_RELOAD_WD));
            rt_mutex_release(&mutex_robot);
            
            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                if(cptWD > 0)  cptWD --;
                rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
                errorCmpt = 0 ;
                rt_mutex_release(&mutex_errorCmpt);                
            }else {
                cptWD ++ ;
                rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
                errorCmpt += 1 ;
                rt_mutex_release(&mutex_errorCmpt);
            }
            
            if(cptWD == 3){
                cout << "COM LOST" << endl << flush;
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(new Message(MESSAGE_ANSWER_COM_ERROR));
                rt_mutex_release(&mutex_monitor);
                
            }
          
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs = robotStarted;
            rt_mutex_release(&mutex_robotStarted);
            
            rt_mutex_acquire(&mutex_modeWD, TM_INFINITE);
            wd = modeWD;
            rt_mutex_release(&mutex_modeWD);
        }
    }
}
/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove, cmpt;
    Message * answer ;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    cout << "BARRIER FOR " << __PRETTY_FUNCTION__ << endl << flush;
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            answer = robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
            if(answer->CompareID(MESSAGE_ANSWER_NACK) || answer->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT) ||
               answer->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND) || answer->CompareID(MESSAGE_ANSWER_ROBOT_ERROR) || answer->CompareID(MESSAGE_ANSWER_COM_ERROR)){
                rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
                errorCmpt += 1 ;
                cmpt = errorCmpt;
                rt_mutex_release(&mutex_errorCmpt);
                if(cmpt == 3){
                    cout << "COM LOST "<<endl<<flush;
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(new Message(MESSAGE_ANSWER_COM_ERROR));
                    rt_mutex_release(&mutex_monitor);
                }
            }else{
                rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
                errorCmpt = 0 ;
                rt_mutex_release(&mutex_errorCmpt);
                cmpt = 0 ;
            }
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

/**
 TODO: Fonctionnalités à implémenter
 */

void Tasks::BatteryTask(void *arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    cout << "BARRIER FOR " << __PRETTY_FUNCTION__ << endl << flush;
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    Message * answer ;
    int status, cmpt;
    
    while(1) {
        rt_task_wait_period(NULL);
        cout << "Periodic battery update"<<endl << flush;
        rt_mutex_acquire(&mutex_openComRobot, TM_INFINITE);
        status = openComRobot;
        rt_mutex_release(&mutex_openComRobot);
        if(status == 1){
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            answer = robot.Write(ComRobot::GetBattery());
            rt_mutex_release(&mutex_robot);
            
            if(answer->CompareID(MESSAGE_ANSWER_NACK) || answer->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT) ||
               answer->CompareID(MESSAGE_ANSWER_ROBOT_UNKNOWN_COMMAND) || answer->CompareID(MESSAGE_ANSWER_ROBOT_ERROR) || answer->CompareID(MESSAGE_ANSWER_COM_ERROR)){
                rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
                errorCmpt += 1 ;
                cmpt = errorCmpt;
                rt_mutex_release(&mutex_errorCmpt);
                
                if(cmpt == 3){
                    cout << "COM LOSTT "<<endl<<flush;
                    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                    monitor.Write(new Message(MESSAGE_ANSWER_COM_ERROR));
                    rt_mutex_release(&mutex_monitor);
                    //rt_sem_v(&sem_kill);
                }
            }else{
                
                rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
                errorCmpt = 0 ;
                rt_mutex_release(&mutex_errorCmpt);
                cmpt = 0 ;
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(answer); // The message is deleted with the Write
                rt_mutex_release(&mutex_monitor);
            
            }
        }
    }
  
}

void Tasks::KillServerTask(void *arg) {
    int err,status;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    while(1){
        rt_sem_p(&sem_kill, TM_INFINITE);
        cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
        // Synchronization barrier (waiting that all tasks are started)

        /**************************************************************************************/
        /* The task server starts here                                                        */
        /**************************************************************************************/

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);

        rt_mutex_acquire(&mutex_errorCmpt, TM_INFINITE);
        errorCmpt = 0;
        rt_mutex_release(&mutex_errorCmpt);

        rt_mutex_acquire(&mutex_openComRobot, TM_INFINITE);
        openComRobot = 0;
        rt_mutex_release(&mutex_openComRobot);

        rt_mutex_acquire(&mutex_modeWD, TM_INFINITE);
        modeWD = 0;
        rt_mutex_release(&mutex_modeWD);

        
               
        monitor.Close();

        cout << " Monitor is lost" << endl << flush;
       
        if (err = rt_mutex_delete(&mutex_monitor) ){
            cerr << "Error mutex delete: " << strerror(-err) << endl << flush;
        }
        
        cout << "monitor KO " << endl << flush;
        
        if (err = rt_mutex_delete(&mutex_robot) ){
            cerr << "Error mutex delete: " << strerror(-err) << endl << flush;
        }
        
        cout << "robot KO " << endl << flush;
        
                
        if (err = rt_mutex_delete(&mutex_robotStarted) ){
            cerr << "Error mutex delete: " << strerror(-err) << endl << flush;
        }
        
        cout << "robotStarted KO " << endl << flush;
        
        if (err = rt_mutex_delete(&mutex_errorCmpt) ){
            cerr << "Error mutex delete: " << strerror(-err) << endl << flush;
        }

        cout << "errorCmpt KO " << endl << flush;

        if (err = rt_mutex_delete(&mutex_openComRobot) ){
            cerr << "Error mutex delete: " << strerror(-err) << endl << flush;
        }
       
        cout << "openCom KO " << endl << flush;
        
        if (err = rt_mutex_delete(&mutex_modeWD) ){
            cerr << "Error mutex delete: " << strerror(-err) << endl << flush;
        }

        cout << "WD KO " << endl << flush;
        
        if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        }
        if (err = rt_mutex_create(&mutex_robot, NULL)) {
            cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        }
        if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
            cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        }
        if (err = rt_mutex_create(&mutex_move, NULL)) {
            cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        }
        if (err = rt_mutex_create(&mutex_openComRobot, NULL)) {
            cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        }
        if (err = rt_mutex_create(&mutex_modeWD, NULL)) {
            cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        }
        if (err = rt_mutex_create(&mutex_errorCmpt, NULL)) {
            cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        }
                
        
        status = monitor.Open(SERVER_PORT);
        
        cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

        if (status < 0) throw std::runtime_error {
            "Unable to start server on port " + std::to_string(SERVER_PORT)
        };
        
        monitor.AcceptClient(); // Wait the monitor client
        cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
        rt_sem_broadcast(&sem_serverOk);

   }
}
//demander prio threads
//demander mutex threads ---> lookup for any mutex use and the functions they call to know when to use them