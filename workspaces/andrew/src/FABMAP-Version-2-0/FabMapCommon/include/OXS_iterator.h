#ifndef OXS_iterator_h
#define OXS_iterator_h

#include <string>
#include <TypeDefinitions.h>
#include "OXS_parser_helper.h"

//Base class for iterators that provide HasNext()
template <class T>
class JavaStyleIterator
{
public:
    virtual ~JavaStyleIterator() {}
    virtual bool HasNext() const = 0;                //returns true when the iterator has not reached the end of the collection.
    virtual void operator++ () = 0;                  //When HasNext() is true, advances the iterator to the next object
    virtual const T& operator* () const = 0;         //returns current object
};

//Null iterator. A base for lists of concatenated iterators.
template <class T>
class NullIterator : public JavaStyleIterator<T>
{
public:
    NullIterator() {}
    virtual ~NullIterator() {}
    virtual bool HasNext() const {return false;}
    //Other operators should never be called.
    virtual void operator++ () { cerr << endl << "ERROR. NullIterator was incremented!" << endl;}
    virtual const T& operator* () const 
    {
        cerr << endl << "ERROR. NullIterator was dereferenced!" << endl;
        return Dummy;
    }
private:
    T Dummy;
};

//Takes two iterators A and B, and creates a new iterator over the concatenation of A and the B
template <class T>
class ConcatenateIterator : public JavaStyleIterator<T>     
{
public:
    ConcatenateIterator(JavaStyleIterator<T>* i_A,JavaStyleIterator<T>* i_B)
        : A(i_A), B(i_B), bUseA(i_A->HasNext())
    {}

    virtual ~ConcatenateIterator() {}

    virtual bool HasNext() const {  return A->HasNext() || B->HasNext();  }

    virtual void operator++ ()
    {
        if(bUseA)
        {
            ++(*A);
            bUseA = A->HasNext();
        }
        else
        {
            ++(*B);
        }
    }

    virtual const T& operator* () const
    {
        if(bUseA)
            return *(*A);
        else
            return *(*B);
    }

private:
    bool bUseA;
    JavaStyleIterator<T>* A;
    JavaStyleIterator<T>* B;
};

//Iterator for general OXS meta-files.
//Accepts as input an OXS file path and returns an iterator over the scenes in that file
//The OXS meta-file may specify other OXS files or OXS meta-files. This is handled transparently by the iterator.
//Scenes are read from disk only when requested from the iterator
class OXSIterator : public JavaStyleIterator<SceneRecord>
{
public:
    OXSIterator(){}
    OXSIterator(string sScenesDir,string sScenesFile,const InterestPointIsInExcludedRegionCheck &IPExclusionCheck, const bool bLoadGeometry, const double dfBlobThreshold);
    virtual ~OXSIterator();

    virtual const SceneRecord& operator* () const { return *(*m_Iterator); }//Dereference encapsulated iterator
    virtual void operator++ () {  ++(*m_Iterator); }                        //Increment encapsulatd iterator
    virtual bool HasNext() const { return m_Iterator->HasNext(); }

protected:
    void SetupMembers(string sScenesDir,string sScenesFile);
    void RepairPaths(string &sScenesDir,string &sScenesFile);

    std::ifstream m_ScenesFile;   
    InterestPointIsInExcludedRegionCheck m_IPExclusionCheck;
    bool m_bNewOXSFormat;
    bool m_bFiltersApplied;
    bool m_bFilterBlobResponse;
    bool m_bLoadGeometry;
    double m_dfBlobThreshold;
    unsigned int m_nLadybugImageWidth;

private:
    JavaStyleIterator<SceneRecord>* m_Iterator;

    vector<JavaStyleIterator<SceneRecord>*> m_ManagedPointers;
};

//Iterator for a single OXS file.
class OXSIterator_SingleFile : public OXSIterator
{
public:
    OXSIterator_SingleFile(const std::string sScenesDir,const std::string sScenesFile,const InterestPointIsInExcludedRegionCheck &IPExclusionCheck, const bool bLoadGeometry, const double dfBlobThreshold);

    virtual ~OXSIterator_SingleFile();

    virtual void operator++ () { FetchNextRecord(); }

    virtual const SceneRecord& operator* () const { return m_CurrentScene; }

    virtual bool HasNext() const { return m_ScenesFile.is_open(); }

protected:
    SceneRecord m_CurrentScene;
    ReadRecord* SceneRecordReader;
    void FetchNextRecord();
};

#endif //OXS_iterator_h
