import numpy as np

class Config:
    # variables
    def __init__(self):
        # load
        try:
            fileContent = open('./src/init/config.conf','r')
            # parser function
            dataToExtract = ['ip', 'port', 'update frequency', 'time out of driving robot', 'transformation']
            self.parseTextFromConfig(fileContent, dataToExtract)
            #for line in f.readlines():data.append(line.replace('\n','').split(' '))
        except:
            self.errorMsg = 'Config file not found. Default values have been set.'
        # checks if variables are set from config file
        # communication
        if not hasattr(self, 'host'):
            self.host = '192.168.1.107' # check ip in cmd with ipconfig
        if not hasattr(self, 'port'):
            self.port = 1238            # port of server
        if not hasattr(self, 'freq'):
            self.freq = 20              # [Hz] update rate
        if not hasattr(self, 'time out of driving robot'):
            self.timeOutDR = 1            # [s] elapsed time until test is interrupted

        # variables AK communication
        self.tic = 0                # [s] time
        # variables Log file
        self.idxCSV = 1
        self.idxRow = 0
        self.prefixName = ''
        self.logArray = np.array([])
        self.newLog = True
        self.isTerminate = False

    # config file parsing
    def parseTextFromConfig(self, fileContent, dataToExtract):
        rowData = []
        for row in fileContent.readlines():
            rowData.append(row)
        for content in dataToExtract:
            for item in rowData:
                try: 
                    item.index(content)
                    idxStr = item.index('= ') + 2
                    try:
                        idxEnd = item.index('\n')
                    except:
                        idxEnd = len(item)   
                    match content:
                        case 'ip':
                            self.host = item[idxStr:idxEnd]
                        case 'port':
                            self.port = int(item[idxStr:idxEnd])
                        case 'update frequency':
                            self.freq = item[idxStr:idxEnd]
                        case 'time out of driving robot':
                            self.timeOut = item[idxStr:idxEnd]
                        case 'transformation':
                            self.ENU2Local = float(item[idxStr:idxEnd])
                except:
                    pass
