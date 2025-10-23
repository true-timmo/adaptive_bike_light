#include <SPI.h>
#include <MFRC522.h>

constexpr uint8_t PIN_SCK  = 18;
constexpr uint8_t PIN_MISO = 19;
constexpr uint8_t PIN_MOSI = 23;
constexpr uint8_t PIN_SS   = 5;  
constexpr uint8_t PIN_RST  = 22;

class ChipReader {
    private:
        MFRC522 mfrc;
        MFRC522::MIFARE_Key defaultKey;

    public:
        ChipReader() :
            mfrc((PIN_SS, PIN_RST))
        {};

        void init() {
            SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

            mfrc.PCD_Init();
            Serial.println("Warte auf RFID-Chip...");

            for (uint8_t i = 0; i < 6; i++) defaultKey.keyByte[i] = 0xFF;
        };

        void read() {
            // Prüfen, ob eine Karte erkannt wurde
            if (!mfrc.PICC_IsNewCardPresent()) return;

            // Versuchen, UID der Karte auszulesen
            if (!mfrc.PICC_ReadCardSerial()) return;

            Serial.print("Karte erkannt! UID: ");
            for (byte i = 0; i < mfrc.uid.size; i++) {
                Serial.printf("%02X ", mfrc.uid.uidByte[i]);
            }
            Serial.println();

            // Karten-Typ anzeigen
            MFRC522::PICC_Type piccType = mfrc.PICC_GetType(mfrc.uid.sak);
            Serial.print("Typ: ");
            Serial.println(mfrc.PICC_GetTypeName(piccType));

            // Karte "freigeben"
            mfrc.PICC_HaltA();
        }

        int readBlock(int blockNumber, byte arrayAddress[]) 
            {
            int largestModulo4Number=blockNumber/4*4;
            int trailerBlock=largestModulo4Number+3;//determine trailer block for the sector

            /*****************************************authentication of the desired block for access***********************************************************/
            MFRC522::StatusCode status = mfrc.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &defaultKey, &(mfrc.uid));
            //byte PCD_Authenticate(byte command, byte blockAddr, MIFARE_Key *key, Uid *uid);
            //this method is used to authenticate a certain block for writing or reading
            //command: See enumerations above -> PICC_CMD_MF_AUTH_KEY_A = 0x60 (=1100000),    // this command performs authentication with Key A
            //blockAddr is the number of the block from 0 to 15.
            //MIFARE_Key *key is a pointer to the MIFARE_Key struct defined above, this struct needs to be defined for each block. New cards have all A/B= FF FF FF FF FF FF
            //Uid *uid is a pointer to the UID struct that contains the user ID of the card.
            if (status != MFRC522::STATUS_OK) {
                    Serial.print("PCD_Authenticate() failed (read): ");
                    Serial.println(mfrc.GetStatusCodeName(status));
                    return 3;//return "3" as error message
            }
            //it appears the authentication needs to be made before every block read/write within a specific sector.
            //If a different sector is being authenticated access to the previous one is lost.


            /*****************************************reading a block***********************************************************/
                    
            byte buffersize = 18;//we need to define a variable with the read buffer size, since the MIFARE_Read method below needs a pointer to the variable that contains the size... 
            status = mfrc.MIFARE_Read(blockNumber, arrayAddress, &buffersize);//&buffersize is a pointer to the buffersize variable; MIFARE_Read requires a pointer instead of just a number
            if (status != MFRC522::STATUS_OK) {
                    Serial.print("MIFARE_read() failed: ");
                    Serial.println((const char*)mfrc.GetStatusCodeName(status));
                    return 4;//return "4" as error message
            }
            Serial.println("block was read");

            return 0;
            }
};