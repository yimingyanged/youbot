#include "OXS_parser.h"
#include "OXS_parser_helper.h"
#include "OXS_iterator.h"
using namespace std;

OXSIterator::OXSIterator(string sScenesDir,string sScenesFile,
                         const InterestPointIsInExcludedRegionCheck &IPExclusionCheck,
                         const bool bLoadGeometry,
                         const double dfBlobThreshold)
: m_bLoadGeometry(bLoadGeometry), m_dfBlobThreshold(dfBlobThreshold), m_IPExclusionCheck(IPExclusionCheck)
{
    RepairPaths(sScenesDir,sScenesFile);
    SetupMembers(sScenesDir,sScenesFile);

    if(m_bNewOXSFormat && bLoadGeometry)
        cout << "Ready to read scenes info and geometry from file...";
    else
        cout << "Ready to read scenes info from file...";

    JavaStyleIterator<SceneRecord>* newIterator;
    //Now, go see if it's a simple OXS file or a meta-file
    std::string sLine;
    if(!m_ScenesFile.eof())
    {
        //grab a line
        getline(m_ScenesFile,sLine,'\n');
        size_t nPos = sLine.find("_META_OXS_FILE_");
        if(nPos!=std::string::npos)
        {
			cout << "Reading MetaOXS file..." << endl;
			//It's a meta-OXS file, which specifies several other OXS files to load
            newIterator = new NullIterator<SceneRecord>();
            m_ManagedPointers.push_back(newIterator);
            m_Iterator = newIterator;
			while(!m_ScenesFile.eof())
			{
				getline(m_ScenesFile,sLine,'\n');
				size_t nPos = sLine.find("File=");					//Look for the keyword we want
				if(nPos!=std::string::npos)
				{
					std::string sFilename;
		            MOOSRemoveChars(sLine,"\r");				    //Defend against Linux/Windows line ending differences
					MOOSValFromString(sFilename,sLine,"File",true);
					//Recursively read from subfiles
                    newIterator = new OXSIterator(sScenesDir,sFilename,IPExclusionCheck,bLoadGeometry,dfBlobThreshold);
                    m_ManagedPointers.push_back(newIterator);
                    newIterator = new ConcatenateIterator<SceneRecord>(m_Iterator,newIterator);
                    m_ManagedPointers.push_back(newIterator);
                    m_Iterator = newIterator;
				}
			}
		}
		else
		{
			//It's just a standard OXS file. Read it in.
			newIterator = new OXSIterator_SingleFile(sScenesDir,sScenesFile,IPExclusionCheck,bLoadGeometry,dfBlobThreshold);
            m_ManagedPointers.push_back(newIterator);
            m_Iterator = newIterator;
		}
	}
}

void OXSIterator::RepairPaths(string &sScenesDir,string &sScenesFile)
{
    //Remove any directory part from sScenesFile and append to sScenesDir
    string sDirPart,sFilePart,sExt;
    MOOSFileParts(sScenesFile,sDirPart,sFilePart,sExt);
    if(!sDirPart.empty())
        sScenesDir += sDirPart + "/";
    sScenesFile = sFilePart + "." + sExt;
}

void OXSIterator::SetupMembers(string sScenesDir,string sScenesFile)
{
    m_nLadybugImageWidth = 384;     //Ladybug x-coords are relative to the side of the image. Use this value to convert to absolute composite image coords. Quick hack...

	const string sScenesFilename = sScenesDir+sScenesFile;

    m_ScenesFile.open(sScenesFilename.c_str());

    if(!m_ScenesFile.is_open())
        cerr << endl << "ERROR: Cannot open:" << endl << sScenesFilename << endl;

    m_bNewOXSFormat = IsOXSNewFormat(sScenesDir,sScenesFile);       //Check the format - is this an old OXS file with wordIDs only, or a new one with geometric info too
    m_bFilterBlobResponse = m_bNewOXSFormat && m_dfBlobThreshold > 0.0;
    m_bFiltersApplied = m_bFilterBlobResponse || !m_IPExclusionCheck.empty();
}

OXSIterator::~OXSIterator()
{
    for(unsigned int i=0;i<m_ManagedPointers.size();i++)
        delete m_ManagedPointers[i];
}


OXSIterator_SingleFile::OXSIterator_SingleFile(const string sScenesDir,const string sScenesFile, const InterestPointIsInExcludedRegionCheck &IPExclusionCheck, const bool bLoadGeometry, const double dfBlobThreshold)
{
    //Setup members.
    m_bLoadGeometry = bLoadGeometry; 
    m_dfBlobThreshold = dfBlobThreshold; 
    m_IPExclusionCheck = IPExclusionCheck;
    SetupMembers(sScenesDir,sScenesFile);

    SceneRecordReader = new ReadRecord(m_ScenesFile,m_bLoadGeometry,m_bNewOXSFormat,m_bFiltersApplied,m_bFilterBlobResponse,m_dfBlobThreshold,m_IPExclusionCheck,m_nLadybugImageWidth);

    //Get the first record.
    FetchNextRecord();
}

OXSIterator_SingleFile::~OXSIterator_SingleFile()
{
    delete SceneRecordReader; 
}


void OXSIterator_SingleFile::FetchNextRecord()
{
    //Go read the next scene record
    if(!(SceneRecordReader->operator () (m_CurrentScene)))
    {
        //If there are none left, close the file
        m_ScenesFile.close();
    }
}
