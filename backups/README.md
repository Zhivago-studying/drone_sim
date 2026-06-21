# DGO 原始节点备份

`DGO_original_d615d70.tar.gz` 保存修改前 Git 基线中的 DGO 节点：

- 来源提交：`d615d70f0704746ce98c53701ff06757a4aff1ef`
- 来源路径：`src/algorithm/src/DGO.cpp`
- 原文件行数：1205
- 原文件 SHA-256：`b91552ab082f8f93fc0f2eeaddc5ef540a1b02c10b06936eccba0725a9ddb1a6`

查看归档内容：

```bash
tar -tzf backups/DGO_original_d615d70.tar.gz
```

恢复到临时目录进行比较：

```bash
mkdir -p /tmp/dgo_original
tar -xzf backups/DGO_original_d615d70.tar.gz -C /tmp/dgo_original
diff -u /tmp/dgo_original/src/algorithm/src/DGO.cpp src/algorithm/src/DGO.cpp
```

不要直接覆盖当前节点；恢复前应先比较差异。
