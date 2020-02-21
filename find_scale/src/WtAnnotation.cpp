#include "WtAnnotation.h"
#include "tinyxml2.h"
#include <dirent.h>
#include "ini_file.h"

#define DEBUG(format, ...) printf("<debug> %s line-%04d: ", basename(__FILE__), __LINE__),printf(format"\n", ##__VA_ARGS__)

#define ASSERT_RETURN(expr) \
{ \
    if ((expr) == false) \
    { \
        printf("Assert: file=%s, func=%s, line=%04d, fail expr: %s\n", \
               basename(__FILE__), __FUNCTION__, __LINE__, #expr); \
        return; \
    } \
}

bool WtAnnotation::LoadRoi(const std::string &sRoiFile, cv::Rect &Roi)
{
    Roi = cv::Rect(0,0,0,0);
    
    const std::string extension_name = sRoiFile.substr(sRoiFile.find_last_of('.'));

    if (extension_name != ".ini" && extension_name != ".xml")
    {
        DEBUG("extension_name is \"%s\" which is illegal!", extension_name.c_str());
    }

    if (extension_name == ".ini")
    {
        unsigned long vertex_size = 8;
        int vertex[8] = {0};
        if (!read_profile_int_vector("other", "prebox", ',', vertex, &vertex_size, sRoiFile.c_str()))
        {
            DEBUG("read prebox failed");
            return false;
        }

        if (vertex_size != 8)
        {
            DEBUG("vertex size:%lu abnormal", vertex_size);
            return false;
        }

        int xmin = vertex[0], ymin = vertex[1];
        int xmax = vertex[0], ymax = vertex[1];
        for (int i = 0; i < vertex_size; i+=2)
        {
            xmin = std::min(xmin, vertex[i]);
            xmax = std::max(xmax, vertex[i]);
        }
        for (int i = 1; i < vertex_size; i+=2)
        {
            ymin = std::min(ymin, vertex[i]);
            ymax = std::max(ymax, vertex[i]);
        }

        cv::Point top_left     = cv::Point(xmin, ymin);
        cv::Point bottom_right = cv::Point(xmax, ymax);
        Roi = cv::Rect(top_left, bottom_right);
        return true;
    }

    std::vector<std::string> names;
    std::vector<cv::Rect> locations;
    WtAnnotation::Load(sRoiFile, names, locations);
    if (names.size() != locations.size() || (int)locations.size() == 0)
    {
        Roi = cv::Rect(0,0,0,0);
        DEBUG("read preset annotation(xml) failed, file name: %s", sRoiFile.c_str());
        return false;
    }

    Roi = locations[0];
    return true;
}

void WtAnnotation::Load(const std::string &path, cv::Mat &image, std::vector<std::string> &names, std::vector<cv::Rect> &locations)
{
    Load(path, names, locations);
    Load(path, image);
}

void WtAnnotation::Load(const std::string &path, cv::Mat &image)
{
    tinyxml2::XMLDocument docXml;
    tinyxml2::XMLError errXml = docXml.LoadFile(path.c_str());

    ASSERT_RETURN (errXml == tinyxml2::XML_SUCCESS);

    // 读取root
    const tinyxml2::XMLElement *root = docXml.FirstChildElement();
    ASSERT_RETURN (root != nullptr);

    // 读取image path
    ASSERT_RETURN (root->FirstChildElement("filename") != nullptr);
    ASSERT_RETURN (root->FirstChildElement("filename")->GetText() != nullptr);

#if defined(_WIN64)||defined(_WIN32)
    const std::string img_dir = path.substr(0, path.find_last_of("\\"));
#elif defined(__linux)||defined(__linux__)||defined(__unix)
    const std::string img_dir = path.substr(0, path.find_last_of('/'));
#endif
    const std::string img_name = root->FirstChildElement("filename")->GetText();
    const std::string img_path = img_dir + "/" + img_name;

    image = cv::imread(img_path, cv::IMREAD_COLOR);

    // 读取image path
//    ASSERT_RETURN (root->FirstChildElement("path") != NULL);
//    ASSERT_RETURN (root->FirstChildElement("path")->GetText() != NULL);
//    image = cv::imread(root->FirstChildElement("path")->GetText(), cv::IMREAD_COLOR);
}

void WtAnnotation::Load(const std::string &path, std::vector<std::string> &names, std::vector<cv::Rect> &locations)
{
    names.clear();
    locations.clear();

    tinyxml2::XMLDocument docXml;
    tinyxml2::XMLError errXml = docXml.LoadFile(path.c_str());

    ASSERT_RETURN(errXml == tinyxml2::XML_SUCCESS)

    // 读取root
    const tinyxml2::XMLElement *root = docXml.FirstChildElement();

    // 遍历object元素
    for (const tinyxml2::XMLElement *object = root->FirstChildElement("object");
         object != nullptr;
         object = object->NextSiblingElement("object"))
    {
        const tinyxml2::XMLElement *name   = object->FirstChildElement("name");
        const tinyxml2::XMLElement *bndbox = object->FirstChildElement("bndbox");

        if (nullptr == name || nullptr == bndbox)
            continue;

        const tinyxml2::XMLElement *xmin = bndbox->FirstChildElement("xmin");
        const tinyxml2::XMLElement *ymin = bndbox->FirstChildElement("ymin");
        const tinyxml2::XMLElement *xmax = bndbox->FirstChildElement("xmax");
        const tinyxml2::XMLElement *ymax = bndbox->FirstChildElement("ymax");

        if (nullptr == xmin || nullptr == ymin || nullptr == xmax || nullptr == ymax)
            continue;

        std::string str = name->GetText();
        names.emplace_back(str.empty() ? str : "NULL");

        cv::Point top_left     = cv::Point(xmin->IntText(), ymin->IntText());
        cv::Point bottom_right = cv::Point(xmax->IntText(), ymax->IntText());
        locations.emplace_back ( cv::Rect(top_left, bottom_right) );
    }
}

