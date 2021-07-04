# Robocon_CV

## 配置开机启动脚本
```sh
    sudo cp /home/domino/robocon/run_domino.service /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable run_domino.service
    sudo systemctl start run_domino.service
    sudo systemctl status run_domino.service
```
