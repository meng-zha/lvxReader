#include <fstream>
#include <stdint.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#pragma pack(1)

typedef struct
{
    uint8_t signature[16];
    uint8_t version[4];
    uint32_t magicCode;
} LvxFileHeader;

typedef struct
{
    uint8_t lidarBroadcastCode[16];
    uint8_t hubBroadcastCode[16];
    uint8_t deviceIndex;
    uint8_t deviceType;
    float roll;
    float pitch;
    float yaw;
    float x;
    float y;
    float z;
} LvxDeviceInfo;

typedef struct
{
    float x;              /**< X axis, Unit:m */
    float y;              /**< X axis, Unit:m */
    float z;              /**< X axis, Unit:m */
    uint8_t reflectivity; /**< Reflectivity */
} LivoxPoint;

typedef struct
{
    uint8_t deviceIndex;
    uint8_t version;
    uint8_t portId;
    uint8_t lidarIndex;
    uint8_t rsvd;
    uint32_t errorCode;
    uint8_t timestampType;
    uint8_t dataType;
    uint8_t timestamp[8];
    LivoxPoint point[100];
} LvxBasePackDetail;

typedef struct
{
    uint64_t currentOffset;
    uint64_t nextOffset;
    uint64_t frameIndex;
    uint64_t packageCount;
} FrameHeader;

#pragma pack()

class LvxReader
{
private:
    /* data */
    std::string fileName;
    std::ifstream lvxFile;
    uint32_t curFrameIndex;
    uint64_t curOffset;

    LvxFileHeader publicHeader;
    uint8_t deviceCount;
    LvxDeviceInfo *deviceInfo;
    std::vector<uint64_t> offsetRecord;

public:
    LvxReader(std::string fileName);
    ~LvxReader();

    void InitLvxFileHeader();
    pcl::PointCloud<pcl::PointXYZI>::Ptr ReadNextFrame(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr SumXSeconds(int time);
};
