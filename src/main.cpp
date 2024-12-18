#include "ArTerabotArm.h" // Biblioteka obs�uguj�ca manipulator TerabotS
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

// Funkcja kontroluj�ca ruch robota
void robotControlLoop(ArRobot& robot, ArTerabotArm& arm, int clientSocket)
{
    char buffer[1024];
    bool manipulatorUsed      = false; // Flaga do pozycji ko�cowej
    bool intermediatePoseUsed = false; // Flaga do pozycji po�redniej

    while (true)
    {
        // Odbieranie danych z serwera
        int bytesReceived = recv(clientSocket, buffer, sizeof(buffer), 0);
        if (bytesReceived < 0)
        {
            throw std::runtime_error("B��d odbierania danych.");
        }
        else if (bytesReceived == 0)
        {
            std::cout << "Po��czenie zako�czone przez serwer.\n";
            break;
        }

        try
        {
            RobotCommand command = parseBuffer(buffer, buffer + bytesReceived);

            double distance = robot.checkRangeDevicesCumulativePolar(-10, 10);

            std::cout << "Otrzymano komend�: ";
            for (const char* it = command.directionStart; it != command.directionEnd; ++it)
            {
                std::cout << *it;
            }
            std::cout << " z odleg�o�ci� dolnej kraw�dzi: " << command.distanceToBottom << " px" << std::endl;
            std::cout << "Odleg�o�� do obiektu: " << distance << " mm" << std::endl;

            if (std::equal(command.directionStart, command.directionEnd, "right"))
            {
                robot.comInt(ArCommands::ENABLE, 1);
                robot.lock();
                robot.setRotVel(-15.0); // Obr�t w prawo
                robot.unlock();
                ArUtil::sleep(500);
            }
            else if (std::equal(command.directionStart, command.directionEnd, "left"))
            {
                robot.comInt(ArCommands::ENABLE, 1);
                robot.lock();
                robot.setRotVel(15.0); // Obr�t w lewo
                robot.unlock();
                ArUtil::sleep(500);
            }
            else if (std::equal(command.directionStart, command.directionEnd, "center"))
            {
                if (distance > 0 && distance < 2500)
                {
                    robot.comInt(ArCommands::ENABLE, 1);
                    robot.lock();
                    robot.setVel(distance / 20); // Zmniejsz pr�dko�� w zale�no�ci od odleg�o�ci
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
                std::cout << "Brak znanej komendy, robot si� obraca." << std::endl;
                robot.comInt(ArCommands::ENABLE, 1);
                robot.lock();
                robot.setRotVel(15.0); // Powolny obr�t
                robot.unlock();
                ArUtil::sleep(500);
            }

            // Zatrzymanie na kr�tki czas
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
                    // Pozycja ko�cowa manipulatora
                    float endPose[5] = {0, -95, -85, 83, 0};
                    arm.enableArm();
                    arm.moveArm(endPose);
                    ArUtil::sleep(5000); // Poczekaj, a� manipulator przyjmie pozycj� ko�cow�

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
                        break; // Zako�czenie p�tli po spe�nieniu warunku
                    }

                    manipulatorUsed      = true;
                    intermediatePoseUsed = false; // Zresetowanie flagi pozycji po�redniej
                }
            }
            // Obs�uga pozycji po�redniej
            else if (distance >= 1300 && distance <= 1900)
            {
                if (!intermediatePoseUsed)
                {
                    std::cout << "Przeszkoda w sredniej odleg�o�ci, przyjecie pozycji posredniej.\n";

                    // Zatrzymanie robota
                    robot.comInt(ArCommands::ENABLE, 1);
                    robot.lock();
                    robot.stop();
                    robot.unlock();
                    ArUtil::sleep(3000);

                    // Pozycja po�rednia manipulatora
                    float intermediatePose[5] = {0, -65, -90, 35, 0};
                    arm.enableArm();
                    arm.moveArm(intermediatePose);
                    ArUtil::sleep(3000);

                    intermediatePoseUsed = true;
                    manipulatorUsed      = false; // Zresetowanie flagi pozycji ko�cowej
                }
            }
            else
            {
                // Reset flag, je�li robot jest daleko od przeszkody
                manipulatorUsed      = false;
                intermediatePoseUsed = false;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "B��d przetwarzania danych: " << e.what() << std::endl;
        }
    }
}

// Obs�uga pozycji ko�cowej
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
        // Pozycja ko�cowa manipulatora
        float endPose[5] = {0, -95, -85, 83, 0};
        arm.enableArm();
        arm.moveArm(endPose);
        ArUtil::sleep(5000); // Poczekaj, a� manipulator przyjmie pozycj� ko�cow�

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
            break; // Zako�czenie p�tli po spe�nieniu warunku
        }

        manipulatorUsed      = true;
        intermediatePoseUsed = false; // Zresetowanie flagi pozycji po�redniej
    }
}
// Obs�uga pozycji po�redniej
else if (distance >= 1300 && distance <= 1900)
{
    if (!intermediatePoseUsed)
    {
        std::cout << "Przeszkoda w sredniej odleg�o�ci, przyjecie pozycji posredniej.\n";

        // Zatrzymanie robota
        robot.comInt(ArCommands::ENABLE, 1);
        robot.lock();
        robot.stop();
        robot.unlock();
        ArUtil::sleep(3000);

        // Pozycja po�rednia manipulatora
        float intermediatePose[5] = {0, -65, -90, 35, 0};
        arm.enableArm();
        arm.moveArm(intermediatePose);
        ArUtil::sleep(3000);

        intermediatePoseUsed = true;
        manipulatorUsed      = false; // Zresetowanie flagi pozycji ko�cowej
    }
}
else
{
    // Reset flag, je�li robot jest daleko od przeszkody
    manipulatorUsed      = false;
    intermediatePoseUsed = false;
}

// Wykonanie odpowiedniego polecenia
if (direction == "right")
{
    robot.comInt(ArCommands::ENABLE, 1);
    robot.lock();
    robot.setRotVel(-15.0); // Obr�t w prawo
    robot.unlock();
    ArUtil::sleep(500);
}
else if (direction == "left")
{
    robot.comInt(ArCommands::ENABLE, 1);
    robot.lock();
    robot.setRotVel(15.0); // Obr�t w lewo
    robot.unlock();
    ArUtil::sleep(500);
}
else if (direction == "center")
{
    if (distance > 0 && distance < 2500)
    {
        robot.comInt(ArCommands::ENABLE, 1);
        robot.lock();
        robot.setVel(distance / 20); // Zmniejsz pr�dko�� w zale�no�ci od odleg�o�ci
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
    robot.setRotVel(15.0); // Powolny obr�t
    robot.unlock();
    ArUtil::sleep(500);
}

// Zatrzymanie na kr�tki czas
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

    // Nawi�zanie po��czenia z manipulatorem
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
    // Zako�czenie pracy z manipulatorem
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

    // Po��czenie z serwerem
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0)
    {
        std::cerr << "Blad polaczenia z serwerem.\n";
        close(clientSocket);
        return 1;
    }

    // Uruchomienie p�tli kontrolnej robota
    robotControlLoop(robot, arm, clientSocket);

    // Zamkni�cie po��czenia i zako�czenie pracy robota
    close(clientSocket);
    // Zako�czenie pracy z manipulatorem
    arm.halt();
    arm.disableArm();
    arm.powerOff();
    Aria::exit(0);
    return 0;
}