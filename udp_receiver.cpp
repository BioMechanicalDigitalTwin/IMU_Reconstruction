#include "udp_receiver.h"
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

void udpReceiver(SensorManager& sensorManager)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5005);
    addr.sin_addr.s_addr = INADDR_ANY;
    
    if(bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return;
    }
    
    std::cout << "Listening on UDP 5005\n";
    
    char buffer[1024];
    
    while(true)
    {
        int len = recv(sockfd, buffer, sizeof(buffer)-1, 0);
        
        if(len <= 0)
            continue;
        
        buffer[len] = '\0';
        
        try
        {
            std::stringstream ss(buffer);
            std::string token;
            
            getline(ss, token, ',');
            
            bool isLFA   = (token == "L_FA");   
            bool isRFA   = (token == "R_FA");   
            bool isLUA   = (token == "L_UA");
            bool isRUA   = (token == "R_UA");   
            bool isLTH   = (token == "L_TH");
            bool isLSH   = (token == "L_SH");
            bool isRTH   = (token == "R_TH");
            bool isRSH   = (token == "R_SH");

            if (!isLFA && !isRFA && !isLUA && !isRUA &&
                !isLTH  && !isLSH   && !isRTH && !isRSH)
                continue;
            
            float w, x, y, z;
            
            getline(ss, token, ','); w = std::stof(token);
            getline(ss, token, ','); x = std::stof(token);
            getline(ss, token, ','); y = std::stof(token);
            getline(ss, token, ','); z = std::stof(token);
            
            glm::quat q = glm::normalize(glm::quat(w, x, y, z));
            
            if      (isLFA)   sensorManager.setLFAQuat(q);
            else if (isRFA)   sensorManager.setRFAQuat(q);
            else if (isLUA)   sensorManager.setLUAQuat(q);
            else if (isRUA)   sensorManager.setRUAQuat(q);
            else if (isLTH)   sensorManager.setLTHQuat(q);
            else if (isLSH)   sensorManager.setLSHQuat(q);
            else if (isRTH)   sensorManager.setRTHQuat(q);
            else if (isRSH)   sensorManager.setRSHQuat(q);
        }
        catch(...) {}
    }
}