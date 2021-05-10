#include "TCPServer.h"

const int BUFFSIZE = 128;

char TCPServer::msg[MAXPACKETSIZE];
map< int, tuple<string, int, int> > TCPServer::mmap;

int TCPServer::num_client;
mutex TCPServer::mt;
mutex TCPServer::mtSerialData;

extern TCPServer tcp;
extern ProdCons_data pc_data[1];

// heart thread function
void* SocketTask(void* arg) {
    struct timespec req;
    req.tv_sec = 0;
    req.tv_nsec = 1000 * 1000;
    int cnn = 0;
    TCPServer* s = (TCPServer*)arg;
    while (1) {

        map< int, tuple<string, int, int > >::iterator it = s->mmap.begin();

        SendData tmp;
        bool output = 0;

        {
            std::lock_guard<std::mutex> guard(tcp.mtSerialData);
            if(0 == pc_data[0].isUse){
            
                tmp = pc_data[0].data;
                pc_data[0].isUse = 1;
                output = 1;
            }
        }
        if(output == 1){
            string message = "channels: ";
        
            message += to_string(static_cast<float>((tmp.tr_data_systerm_time.d)/1e3));
            message += ',';
            message += to_string(static_cast<float>((tmp.tr_data_chassis_speed_rpm1.d)));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_chassis_speed_rpm2.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_chassis_speed_rpm3.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_chassis_speed_rpm4.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_chassis_total_ecd1.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_chassis_total_ecd2.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_chassis_total_ecd3.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_chassis_total_ecd4.d));
            message += ',';

            message += to_string(static_cast<float>(tmp.tr_data_trace_speed_rpm.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_pitch_speed_rpm.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_load_l_speed_rpm.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_load_r_speed_rpm.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_traction_total_ecd.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_pitch_total_ecd.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_load_l_total_ecd.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_load_r_total_ecd.d));
            message += ',';

            message += to_string(static_cast<float>(tmp.tr_data_act_pos_sys_x.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_act_pos_sys_y.d));
            message += ',';
            message += to_string(static_cast<float>(tmp.tr_data_act_pos_sys_w.d));

            message += '\n';

            for (; it != s->mmap.end(); ) {
                //cout << message << endl;
                send(it->first, message.c_str(), message.length(), 0);
                ++it;
            }
        }
        

        

        nanosleep(&req, NULL);
    }
}

void* TCPServer::Task(void* arg) {
    int n;
    long long new_tmp_fd = (long long)arg;
    int new_fd = new_tmp_fd;
    struct timespec req;
    req.tv_sec = 0;
    req.tv_nsec = 1000 * 1000;

    pthread_detach(pthread_self());

    string ip = get<0>(mmap[new_fd]);
    int uid;

    while (1) {
        n = recv(new_fd, msg, MAXPACKETSIZE, 0);
        if (n != -1) {
            msg[n] = 0;
            //cout << msg << endl;
            lock_guard<mutex> guard(mt);
            if (n == 0) {
                close(new_fd);
                mmap.erase(new_fd);  // 从map中移除该记录
                break;
            }
        }
        nanosleep(&req, NULL);
    }
    pthread_exit(NULL);
    return 0;
}

int TCPServer::setup(int port, vector<int> opts) {
    int opt = 1;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serverAddress, 0, sizeof(serverAddress));

    for (unsigned int i = 0; i < opts.size(); i++) {
        if ((setsockopt(sockfd, SOL_SOCKET, opts.at(i), (char*)&opt, sizeof(opt))) < 0) {
            //cout << "Errore setsockopt";
            return -1;
        }
    }

    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddress.sin_port = htons(port);

    if ((::bind(sockfd, (struct sockaddr*)&serverAddress, sizeof(serverAddress))) < 0) {
        //cout << "Errore bind";
        return -1;
    }

    if (listen(sockfd, 5) < 0) {
        //cout << "Errore listen";
        return -1;
    }
    num_client = 0;

    pthread_t id;
    pthread_create(&id, NULL, SocketTask, (void*)this); // 创建Socket线程

    return 0;
}

void TCPServer::accepted() {
    socklen_t sosize = sizeof(clientAddress);
    int new_fd = accept(sockfd, (struct sockaddr*)&clientAddress, &sosize);
    string new_ip = inet_ntoa(clientAddress.sin_addr);

    mmap.emplace(new_fd, make_tuple(new_ip, 0, num_client));
    pthread_create(&serverThread[num_client], NULL, &Task, (void*)new_fd);
    num_client++;
}

void TCPServer::closed() {
    close(sockfd);
}
