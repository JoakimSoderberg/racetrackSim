#ifndef __DATAHANDLER_TMPL_INCLUDED__
#define __DATAHANDLER_TMPL_INCLUDED__

#include <lcm/lcm-cpp.hpp>
#include <map>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <utility>
#include <sys/poll.h>

/** Usage
 *  //initialization 
 *  IOStream is = msgHandler->registerInputDataStream<Type>(); //global
 *  //during loop
 *  LCMHandler->handle();
 *
 *  Type msg = is->getLatest();
 */

//TODO do not use auto to be able to use in Local classes

class DataHandlerBase {
protected:
  int64_t lastTimestamp;
  int64_t dt;
  std::string const chan;
  DataHandlerBase(std::string chan_arg) : chan(chan_arg), lastTimestamp(-1) {};
    
public:
  virtual ~DataHandlerBase() {}; //must implement virtual destructor or get vtable error!
};

//TODO figure out if we can handle timestamps in a smart way (perhaps cast to a struct)
template <class T>
class DataHandler : public DataHandlerBase {
private:
  T data;
public: //should be private and use friend class (THE USER SHOULD NOT SEE THIS!)
  T getLatest(int64_t & dt) {
    return data;
  };
  void lcmCallback(lcm::ReceiveBuffer const * rbuf,
		   std::string const & chan,
		   T const * msg) {
    data = *msg;
  }
  DataHandler(std::string const chan) : DataHandlerBase(chan) {};
};

//forward declaration
class LCMManager;

template <class T>
class LCMInputStream {
  friend class LCMManager;
private :
  DataHandler<T> & dataHandler_; 
  LCMInputStream(DataHandler<T> & dataHandler) : dataHandler_(dataHandler) {};
public:
  T getLatest(int64_t & dt) {
    return dataHandler_.getLatest(dt);
  }
};

template <class T>
class LCMOutputStream {
  friend class LCMManager;
private :
  std::string chan_;
  lcm::LCM* lcm_;
  LCMOutputStream(std::string chan, lcm::LCM* lcm) : chan_(chan), lcm_(lcm) {};
public:
  void put(T* data) {
    lcm_->publish(chan_,data);
  }
};

//to prevent brain bleed
typedef std::pair<size_t,std::string> dhKey;
//typedef std::shared_ptr<DataHandlerBase> dhValue;
typedef boost::shared_ptr<DataHandlerBase> dhValue;

class LCMManager {
private:
  lcm::LCM lcm_;
  //TODO use unordered map instead
  std::map<dhKey,dhValue> dataHandlers_;
  
  bool lcm_poll(lcm::LCM* lcm) {
    pollfd fds = { lcm->getFileno(), POLLIN, 0}; 
    return ( poll(&fds, 1, 0) >= 0) && (fds.revents == POLLIN);
  };
  int handleLCM(lcm::LCM* lcm) {
    while(lcm_poll(lcm))  { 
      if(lcm->handle() != 0) { //this only takes one message! (send or recieve!)
	return 1;
      }
    }
    return 0;
  };
public:
  LCMManager(std::string lcm_url="") : lcm_(lcm_url) {};
  void manage(){ //TODO name
    
    if(handleLCM(&lcm_) != 0) {
      //TODO error handling
    }
  }; 
  
  //get an appropriate input stream
  template <class T>
  LCMInputStream<T> registerInputDataStream(std::string const chan) {
    auto element_insertion =
      dataHandlers_.insert(std::pair<dhKey,dhValue> (dhKey(typeid(T).hash_code(), chan),
						     dhValue(new DataHandler<T>(chan))));
    
    auto iter = element_insertion.first;
    //std::shared_ptr<DataHandlerBase> val = (*iter).second;
    boost::shared_ptr<DataHandlerBase> val = (*iter).second;
    DataHandlerBase * parentPtr = val.get();
    DataHandler<T> * implementingObjectPtr = (DataHandler<T>*) parentPtr;
    
    if(element_insertion.second) { //an insertion happened
      
      //lcm.subscribe(channel name, function pointer to class function, pointer to class object)
      lcm_.subscribe(chan, &DataHandler<T>::lcmCallback,  implementingObjectPtr);
    }
    return LCMInputStream<T>(*implementingObjectPtr);
  }

  //get an appropriate output stream
  template <class T>
  LCMOutputStream<T> registerOutputDataStream(std::string const chan) {
    return LCMOutputStream<T>(chan,&lcm_);
  }
};

#endif
