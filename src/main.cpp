#include <Settings.h>

void wait(unsigned long milliseconds)
{
    unsigned long timeout = millis() + milliseconds;
    while (millis() < timeout)
    {
        yield();
    }
}

void sendModbusRequest(uint16_t startReg, uint16_t numRegs, const char *ip, int port, uint8_t slave_id)
{
    uint8_t meterRequest[12] = {
        0x00, 0x01,
        0x00, 0x00,
        0x00, 0x06,
        slave_id,
        0x03,
        static_cast<uint8_t>((startReg >> 8) & 0xFF),
        static_cast<uint8_t>(startReg & 0xFF),
        static_cast<uint8_t>((numRegs >> 8) & 0xFF),
        static_cast<uint8_t>(numRegs & 0xFF)};

    if (ethClient.connect(ip, port))
    {
        ethClient.write(meterRequest, sizeof(meterRequest));
    }
    else
    {
        Serial.print("Failed meter IP:PORT");
        Serial.print(ip);
        Serial.print(":");
        Serial.println(port);
        display.fillRect(0, 32, 128, 32, SSD1306_BLACK);
        display.setFont(NULL);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(5, 38);
        display.println("Connect Failed:");
        display.setTextSize(1);
        display.setCursor(5, 50);
        display.print(ip);
        display.print(":");
        display.print(port);
        display.display();
        readSuccess = false;
    }
}

bool readModbusResponse(uint16_t numRegs, float *temp_values)
{
    uint8_t modbusResponse[512] = {0};
    wait(FREQ_READ / NUM_METERS * 1000);

    if (ethClient.available())
    {
        int bytesRead = ethClient.read(modbusResponse, sizeof(modbusResponse));
        if (bytesRead >= 9)
        {
            int byteCount = modbusResponse[8];

            if (byteCount != numRegs * 2)
            {
                Serial.println("Incorrect byte count in response.");
                ethClient.stop();
                return false;
            }

            for (int paramIndex = 0; paramIndex < NUM_PARAMETERS; paramIndex++)
            {
                int registerOffset = parameter_addresses[paramIndex] - parameter_addresses[0];
                int dataIndex = 9 + registerOffset * 2;

                if (dataIndex + 3 >= sizeof(modbusResponse))
                {
                    Serial.println("Data index out of bounds.");
                    continue;
                }

                uint16_t reg1 = (modbusResponse[dataIndex] << 8) | modbusResponse[dataIndex + 1];
                uint16_t reg2 = (modbusResponse[dataIndex + 2] << 8) | modbusResponse[dataIndex + 3];
                int32_t rawValue = ((int32_t)reg1 << 16) | reg2;

                float value = rawValue;

                if (parameter_addresses[paramIndex] >= 776 && parameter_addresses[paramIndex] <= 794 || parameter_addresses[paramIndex] >= 798 && parameter_addresses[paramIndex] <= 814)
                {
                    value = value / 100.0;
                }
                else if (parameter_addresses[paramIndex] >= 768 && parameter_addresses[paramIndex] <= 774 || parameter_addresses[paramIndex] == 796 || parameter_addresses[paramIndex] == 816 || parameter_addresses[paramIndex] == 818 || parameter_addresses[paramIndex] == 820)
                {
                    value = value / 1000.0;
                }

                bool isAverageParam = ((parameter_addresses[paramIndex] >= 776 && parameter_addresses[paramIndex] <= 788) ||
                                       parameter_addresses[paramIndex] == 796 ||
                                       parameter_addresses[paramIndex] == 816 ||
                                       parameter_addresses[paramIndex] == 818 ||
                                       parameter_addresses[paramIndex] == 820);

                if (isAverageParam)
                {
                    temp_values[paramIndex] += value / NUM_METERS;
                }
                else
                {
                    temp_values[paramIndex] += value;
                }
            }
            ethClient.stop();
            return true;
        }
        else
        {
            Serial.println("Error reading Modbus response.");
            ethClient.stop();
            return false;
        }
    }
    else
    {
        Serial.println("No data available from Modbus.");
        ethClient.stop();
        return false;
    }
}

void readAllMeters()
{

    float temp_parameter_values[NUM_PARAMETERS] = {0};

    for (int meterIndex = 0; meterIndex < NUM_METERS; meterIndex++)
    {
        uint16_t startReg = 768;
        uint16_t numRegs = 98;

        sendModbusRequest(startReg, numRegs, modbus_ips[meterIndex], modbus_ports[meterIndex], meter_slave_ids[meterIndex]);

        if (!readModbusResponse(numRegs, temp_parameter_values))
        {
            readSuccess = false;
            break;
        }
        else
        {
            if (meterIndex == NUM_METERS - 1)
            {
                readSuccess = true;
            }
        }
    }

    if (readSuccess)
    {
        for (int i = 0; i < NUM_PARAMETERS; i++)
        {
            parameter_values[i] = temp_parameter_values[i];
        }
        modbus_registers_values[0] = parameter_values[7];
        modbus_registers_values[1] = parameter_values[8];
        modbus_registers_values[2] = parameter_values[9];
        modbus_registers_values[3] = parameter_values[4];
        modbus_registers_values[4] = parameter_values[5];
        modbus_registers_values[5] = parameter_values[6];
        modbus_registers_values[6] = parameter_values[0];
        modbus_registers_values[7] = parameter_values[1];
        modbus_registers_values[8] = parameter_values[2];
        modbus_registers_values[9] = parameter_values[11];
        modbus_registers_values[10] = parameter_values[15];
        modbus_registers_values[11] = parameter_values[16];
        modbus_registers_values[12] = parameter_values[17];
        modbus_registers_values[13] = parameter_values[12];
        modbus_registers_values[14] = parameter_values[13];
        modbus_registers_values[15] = parameter_values[27] + parameter_values[20];
        modbus_registers_values[16] = parameter_values[28] + parameter_values[31];
        modbus_registers_values[17] = parameter_values[27];
        modbus_registers_values[18] = parameter_values[28];
        modbus_registers_values[19] = parameter_values[30];
        modbus_registers_values[20] = parameter_values[31];
        for (int i = 0; i < NUM_REGISTERS; i++)
        {
            float fValue = modbus_registers_values[i];
            float scale = scaling_factors[i];
            fValue *= scale;
            uint32_t intValue;
            memcpy(&intValue, &fValue, sizeof(float));
            uint16_t highWord = (intValue >> 16) & 0xFFFF;
            uint16_t lowWord = intValue & 0xFFFF;
            uint16_t regAddr = modbus_registers_addresses[i];
            mb.Hreg(regAddr, highWord);
            mb.Hreg(regAddr + 1, lowWord);
        }
    }
}

void displaydata()
{
    if (displayParamN < sizeof(parameter_names_disp) / sizeof(parameter_names_disp[0]))
    {
        display.fillRect(0, 32, 128, 32, SSD1306_BLACK);
        display.setFont(&Dialog_plain_12);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(5, 44);
        display.println(parameter_names_disp[displayParamN]);
        display.setTextSize(1);
        display.setCursor(40, 58);
        display.println(parameter_values[displayParamN]);
        display.display();
        displayParamN += 1;
    }
    else
    {
        displayParamN = 0;
    }
}

void loop()
{
}

void Task0code(void *pvParameters)
{
    for (;;)
    {

        readAllMeters();
        if (readSuccess)
        {
            displaydata();
        }
        vTaskDelay(10);
    }
}

void Task1code(void *pvParameters)
{
    for (;;)
    {
        if (readSuccess)
        {
            mb.task();
        }
        vTaskDelay(10);
    }
}

void setup()
{
    Wire.begin();
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        for (;;)
            ;
    }
    display.setRotation(0);
    display.fillRect(0, 0, 128, 64, SSD1306_BLACK);
    display.drawBitmap(0, 0, proso, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
    display.fillRect(0, 32, 128, 32, SSD1306_BLACK);
    display.display();
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1);
    mb.begin(&Serial2);
    mb.slave(MODBUS_ID);

    for (int i = 0; i < NUM_REGISTERS; i++)
    {
        uint16_t regAddr = modbus_registers_addresses[i];
        mb.addHreg(regAddr);
        mb.addHreg(regAddr + 1);
    }

    Ethernet.init(ETHERNET_CS_PIN);
    Ethernet.begin(mac_ethernet, ip, dns, gateway, subnet);
    // if (Ethernet.begin(mac_ethernet) == 0)
    // {
    //     Serial.println("Ethernet initialization failed. Check the connection.");
    //     while (true)
    //         ;
    // }
    Serial.print("Ethernet IP: ");
    Serial.println(Ethernet.localIP());
    display.fillRect(0, 32, 128, 32, SSD1306_BLACK);
    display.setFont(NULL);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(5, 38);
    display.println("My IP Address:");
    display.setTextSize(1);
    display.setCursor(5, 50);
    display.print(Ethernet.localIP());
    display.display();
    wait(FREQ_READ * 1000 / NUM_METERS);
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        Serial.println("Ethernet adapter not found.");
        while (true)
            ;
    }
    if (Ethernet.linkStatus() == LinkOFF)
    {
        Serial.println("Ethernet cable not connected.");
    }
    xTaskCreatePinnedToCore(Task0code, "Task0", 10000, NULL, 1, &Task0, 0);
    xTaskCreatePinnedToCore(Task1code, "Task1", 10000, NULL, 1, &Task1, 1);
}
