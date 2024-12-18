#include "ArTerabotArm.h" // Biblioteka obs³uguj¹ca manipulator TerabotS
#include "Aria.h"
#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

struct RobotCommand
{
    const char* directionStart;
    const char* directionEnd;
    int         distanceToBottom;
};

RobotCommand parseBuffer(const char* begin, const char* end)
{
    RobotCommand command;

    const char* commaPos = std::find(begin, end, ',');
    if (commaPos == end)
    {
        throw std::invalid_argument("Niepoprawny format danych: brak przecinka.");
    }

    command.directionStart = begin;
    command.directionEnd   = commaPos;

    const char* distanceStart = commaPos + 1;
    command.distanceToBottom  = fastStringToInt(distanceStart, end);

    return command;
}

int fastStringToInt(const char* begin, const char* end)
{
    int result = 0;
    while (begin != end)
    {
        if (*begin < '0' || *begin > '9')
        {
            throw std::invalid_argument("Niepoprawny znak w liczbie.");
        }
        result = result * 10 + (*begin - '0');
        ++begin;
    }
    return result;
}

// Funkcja kontroluj¹ca ruch robota
void robotControlLoop(ArRobot& robot, ArTerabotArm& arm, int clientSocket)
{
    char buffer[1024];
    bool manipulatorUsed      = false; // Flaga do pozycji koñcowej
    bool intermediatePoseUsed = false; // Flaga do pozycji poœredniej

    while (true)
    {
        // Odbieranie danych z serwera
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytesReceived < 0)
        {
            throw std::runtime_error("B³¹d odbierania danych.");
        }
        else if (bytesReceived == 0)
        {
            std::cout << "Po³¹czenie zakoñczone przez serwer.\n";
            break;
        }

        try
        {
            RobotCommand command = parseBuffer(buffer, buffer + bytesReceived);

            double distance = robot.checkRangeDevicesCumulativePolar(-10, 10);

            std::cout << "Otrzymano komendê: ";
            for (const char* it = command.directionStart; it != command.directionEnd; ++it)
            {
                std::cout << *it;
            }
            std::cout << " z odleg³oœci¹ dolnej krawêdzi: " << command.distanceToBottom << " px" << std::endl;
            std::cout << "Odleg³oœæ do obiektu: " << distance << " mm" << std::endl;

            if (std::equal(command.directionStart, command.directionEnd, "right"))
            {
                robot.comInt(ArCommands::ENABLE, 1);
                robot.lock();
                robot.setRotVel(-15.0); // Obrót w prawo
                robot.unlock();
                ArUtil::sleep(500);
            }
            else if (std::equal(command.directionStart, command.directionEnd, "left"))
            {
                robot.comInt(ArCommands::ENABLE, 1);
                robot.lock();
                robot.setRotVel(15.0); // Obrót w lewo
                robot.unlock();
                ArUtil::sleep(500);
            }
            else if (std::equal(command.directionStart, command.directionEnd, "center"))
            {
                if (distance > 0 && distance < 2500)
                {
                    robot.comInt(ArCommands::ENABLE, 1);
                    robot.lock();
                    robot.setVel(distance / 20); // Zmniejsz prêdkoœæ w zale¿noœci od odleg³oœci
                    robot.unlock();
                    ArUtil::sleep(1005);
                }
                else
                {
                    robot.comInt(ArCommands::ENABLE, 1);
                    robot.lock();
                    robot.setVel(200); // Ruch do przodu
                    robot.unlock();
                    ArUtil::sleep(1005);
                }
            }
            else if (std::equal(command.directionStart, command.directionEnd, "rotate"))
            {
                std::cout << "Brak znanej komendy, robot siê obraca." << std::endl;
                robot.comInt(ArCommands::ENABLE, 1);
                robot.lock();
                robot.setRotVel(15.0); // Powolny obrót
                robot.unlock();
                ArUtil::sleep(500);
            }

            // Zatrzymanie na krótki czas
            robot.comInt(ArCommands::ENABLE, 1);
            robot.lock();
            robot.stop();
            robot.unlock();
            ArUtil::sleep(100);

            if (distance > 0 && distance < 1300)
            {
                if (!manipulatorUsed)
                {
                    std::cout << "Bliska przeszkoda, zatrzymanie i operacja na manipulatorze.\n";

                    // Zatrzymanie robota
                    robot.comInt(ArCommands::ENABLE, 1);
                    robot.lock();
                    robot.stop();
                    robot.unlock();
                    ArUtil::sleep(3000);
                    // Pozycja koñcowa manipulatora
                    float endPose[5] = {0, -95, -85, 83, 0};
                    arm.enableArm();
                    arm.moveArm(endPose);
                    ArUtil::sleep(5000); // Poczekaj, a¿ manipulator przyjmie pozycjê koñcow¹

                    if (command.distanceToBottom > 0 && command.distanceToBottom < 220)
                    {
                        robot.comInt(ArCommands::ENABLE, 1);
                        robot.lock();
                        robot.stop();
                        robot.unlock();
                        ArUtil::sleep(3000);
                        float pose[5] = {0, -70, -90, 60, 0};
                        arm.enableArm();
                        arm.moveArm(pose);
                        ArUtil::sleep(3000);
                        break; // Zakoñczenie pêtli po spe³nieniu warunku
                    }

                    manipulatorUsed      = true;
                    intermediatePoseUsed = false; // Zresetowanie flagi pozycji poœredniej
                }
            }
            // Obs³uga pozycji poœredniej
            else if (distance >= 1300 && distance <= 1900)
            {
                if (!intermediatePoseUsed)
                {
                    std::cout << "Przeszkoda w sredniej odleg³oœci, przyjecie pozycji posredniej.\n";

                    // Zatrzymanie robota
                    robot.comInt(ArCommands::ENABLE, 1);
                    robot.lock();
                    robot.stop();
                    robot.unlock();
                    ArUtil::sleep(3000);

                    // Pozycja poœrednia manipulatora
                    float intermediatePose[5] = {0, -65, -90, 35, 0};
                    arm.enableArm();
                    arm.moveArm(intermediatePose);
                    ArUtil::sleep(3000);

                    intermediatePoseUsed = true;
                    manipulatorUsed      = false; // Zresetowanie flagi pozycji koñcowej
                }
            }
            else
            {
                // Reset flag, jeœli robot jest daleko od przeszkody
                manipulatorUsed      = false;
                intermediatePoseUsed = false;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "B³¹d przetwarzania danych: " << e.what() << std::endl;
        }
    }
}

// Obs³uga pozycji koñcowej
if (distance > 0 && distance < 1300)
{
    if (!manipulatorUsed)
    {
        std::cout << "Bliska przeszkoda, zatrzymanie i operacja na manipulatorze.\n";

        // Zatrzymanie robota
        robot.comInt(ArCommands::ENABLE, 1);
        robot.lock();
        robot.stop();
        robot.unlock();
        ArUtil::sleep(3000);
        // Pozycja koñcowa manipulatora
        float endPose[5] = {0, -95, -85, 83, 0};
        arm.enableArm();
        arm.moveArm(endPose);
        ArUtil::sleep(5000); // Poczekaj, a¿ manipulator przyjmie pozycjê koñcow¹

        if (command.distanceToBottom > 0 && command.distanceToBottom < 220)
        {
            robot.comInt(ArCommands::ENABLE, 1);
            robot.lock();
            robot.stop();
            robot.unlock();
            ArUtil::sleep(3000);
            float pose[5] = {0, -70, -90, 60, 0};
            arm.enableArm();
            arm.moveArm(pose);
            ArUtil::sleep(3000);
            break; // Zakoñczenie pêtli po spe³nieniu warunku
        }

        manipulatorUsed      = true;
        intermediatePoseUsed = false; // Zresetowanie flagi pozycji poœredniej
    }
}
// Obs³uga pozycji poœredniej
else if (distance >= 1300 && distance <= 1900)
{
    if (!intermediatePoseUsed)
    {
        std::cout << "Przeszkoda w sredniej odleg³oœci, przyjecie pozycji posredniej.\n";

        // Zatrzymanie robota
        robot.comInt(ArCommands::ENABLE, 1);
        robot.lock();
        robot.stop();
        robot.unlock();
        ArUtil::sleep(3000);

        // Pozycja poœrednia manipulatora
        float intermediatePose[5] = {0, -65, -90, 35, 0};
        arm.enableArm();
        arm.moveArm(intermediatePose);
        ArUtil::sleep(3000);

        intermediatePoseUsed = true;
        manipulatorUsed      = false; // Zresetowanie flagi pozycji koñcowej
    }
}
else
{
    // Reset flag, jeœli robot jest daleko od przeszkody
    manipulatorUsed      = false;
    intermediatePoseUsed = false;
}

// Wykonanie odpowiedniego polecenia
if (direction == "right")
{
    robot.comInt(ArCommands::ENABLE, 1);
    robot.lock();
    robot.setRotVel(-15.0); // Obrót w prawo
    robot.unlock();
    ArUtil::sleep(500);
}
else if (direction == "left")
{
    robot.comInt(ArCommands::ENABLE, 1);
    robot.lock();
    robot.setRotVel(15.0); // Obrót w lewo
    robot.unlock();
    ArUtil::sleep(500);
}
else if (direction == "center")
{
    if (distance > 0 && distance < 2500)
    {
        robot.comInt(ArCommands::ENABLE, 1);
        robot.lock();
        robot.setVel(distance / 20); // Zmniejsz prêdkoœæ w zale¿noœci od odleg³oœci
        robot.unlock();
        ArUtil::sleep(1005);
    }
    else
    {
        robot.comInt(ArCommands::ENABLE, 1);
        robot.lock();
        robot.setVel(200); // Ruch do przodu
        robot.unlock();
        ArUtil::sleep(1005);
    }
}
else if (direction == "rotate")
{
    std::cout << "Brak znanej komendy, robot sie obraca." << std::endl;
    robot.comInt(ArCommands::ENABLE, 1);
    robot.lock();
    robot.setRotVel(15.0); // Powolny obrót
    robot.unlock();
    ArUtil::sleep(500);
}

// Zatrzymanie na krótki czas
robot.comInt(ArCommands::ENABLE, 1);
robot.lock();
robot.stop();
robot.unlock();
ArUtil::sleep(100);
}
}

int main(int argc, char** argv)
{
    float defaultJointSpeed = 15;
    Aria::init();
    ArArgumentParser argParser(&argc, argv);
    argParser.loadDefaultArguments();
    ArRobot          robot;
    ArRobotConnector robotConnector(&argParser, &robot);
    ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);
    argParser.addDefaultArgument("-connectLaser");
    if (!robotConnector.connectRobot())
    {
        ArLog::log(ArLog::Terse, "Could not connect to the robot.");
        if (argParser.checkHelpAndWarnUnparsed())
        {
            Aria::logOptions();
            Aria::exit(1);
        }
    }
    if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
    {
        Aria::logOptions();
        Aria::exit(1);
    }
    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);
    robot.runAsync(true);
    laserConnector.connectLasers();
    ArUtil::sleep(400);

    // Tworzenie obiektu manipulatora TerabotS

    ArTerabotArm arm(&robot, "/dev/ttyS1");

    // Nawi¹zanie po³¹czenia z manipulatorem
    if (!arm.open())
    {
        std::cerr << "Nie udalo sie nawiazac polaczenia z manipulatorem." << std::endl;
        Aria::exit(1);
    }
    arm.powerOn();
    arm.reset();
    arm.enableArm();
    arm.setAllJointSpeeds(defaultJointSpeed);
    ArUtil::sleep(2000);
    float pose[5] = {0, -70, -90, 60, 0};
    arm.moveArm(pose);
    ArUtil::sleep(5000);
    // Zakoñczenie pracy z manipulatorem
    // arm.halt();
    // arm.disableArm();
    // arm.powerOff();

    // Konfiguracja socketu klienta
    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket < 0)
    {
        std::cerr << "Blad tworzenia socketu.\n";
        return 1;
    }

    struct sockaddr_in serverAddress;
    serverAddress.sin_family      = AF_INET;
    serverAddress.sin_port        = htons(12343);              // Port serwera
    serverAddress.sin_addr.s_addr = inet_addr("10.0.125.122"); // IP laptopa (serwera)

    // Po³¹czenie z serwerem
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0)
    {
        std::cerr << "Blad polaczenia z serwerem.\n";
        close(clientSocket);
        return 1;
    }

    // Uruchomienie pêtli kontrolnej robota
    robotControlLoop(robot, arm, clientSocket);

    // Zamkniêcie po³¹czenia i zakoñczenie pracy robota
    close(clientSocket);
    // Zakoñczenie pracy z manipulatorem
    arm.halt();
    arm.disableArm();
    arm.powerOff();
    Aria::exit(0);
    return 0;
}