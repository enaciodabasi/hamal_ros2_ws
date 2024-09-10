extern crate yaml_rust;
use std::{env, fs::File, io::Read};

use yaml_rust::{yaml, Yaml, YamlLoader};

#[derive(Default)]
struct ServiceTemplateParams {
  file_name: String,
  ros_domain_id: i32,
  

}

impl ServiceTemplateParams{

  fn new() -> Self {
    ServiceTemplateParams { ..Default::default() }
  }

  fn parse_docs(&mut self, doc: &yaml::Yaml)
  {
    let file_name = doc["file_name"].as_str();
    
  } 

}

fn main() {
  let args: Vec<_> = env::args().collect();
  let file_read_res = File::open(&args[1]);
  let mut file_as_str: String = String::new();
  match file_read_res {
    Ok(mut f) => f.read_to_string(&mut file_as_str).unwrap(),
    Err(_e) => return,
  };
  
  let yaml_doc_res = yaml::YamlLoader::load_from_str(&file_as_str);
  let mut yaml_docs: Vec<yaml_rust::Yaml> = Vec::new();
  match yaml_doc_res {
    Ok(docs) => yaml_docs = docs,
    Err(_e) => return
  }
  
  
  for doc in &yaml_docs {

    let mut params: ServiceTemplateParams = ServiceTemplateParams::new();
    params.parse_docs(doc);

  } 
  

}
